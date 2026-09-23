/*
 * Projeto: Autopiloto didatico de Drone - Sistemas em Tempo Real
 * Plataforma: ESP32 + FreeRTOS (ESP-IDF 6.1)
 *
 * Objetivo:
 *   Simular tarefas tipicas de um drone e medir comportamento temporal
 *   sob diferentes politicas de prioridade, frequencias de CPU e modos
 *   de escalonamento.
 *
 * Tarefas principais:
 *   - FUS_IMU  : fusao inercial periodica (T = 5 ms, D = 5 ms)
 *   - CTRL_ATT : controle de atitude, disparado pela FUS_IMU (D = 5 ms)
 *   - NAV_PLAN : navegacao/telemetria acionada pelos Touch B e C (D = 20 ms)
 *   - FS_TASK  : fail-safe acionado pelo Touch D via interrupcao (D = 10 ms)
 *
 * Instrumentacao:
 *   O programa mede WCET observado, jitter, latencia, tempo de resposta,
 *   deadline misses e ocupacao estimada das tarefas do drone.
 *
 * Observacao:
 *   Todas as tarefas do experimento sao fixadas no core 0.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/touch_sens.h"

// ==========================================================
// CONFIGURAÇÕES DO EXPERIMENTO
// ==========================================================

#define TAG "DRONE_RTS"

// Periodo da tarefa periodica FUS_IMU e duracao de cada cenario de teste.
#define FUS_IMU_PERIOD_MS   5
#define TEST_DURATION_MS    30000


// Touch B, C e D
#define TOUCH_NAV_CH   7       // T7 = GPIO27
#define TOUCH_TEL_CH   8       // T8 = GPIO33
#define TOUCH_FS_CH    9       // T9 = GPIO32

// Politica de prioridade selecionada para este build.
// CUSTOM prioriza primeiro o fail-safe, por ser a funcao de seguranca.
#define POLICY_NAME "CUSTOM"

// No FreeRTOS, numeros maiores representam prioridades maiores.
// A tarefa REPORT fica com menor prioridade para nao interferir no experimento.
#define PRIO_FS_TASK      5
#define PRIO_FUS_IMU      4
#define PRIO_CTRL_ATT     3
#define PRIO_NAV_PLAN     2
#define PRIO_REPORT       1
#define TASK_STACK_SIZE   3072
#define REPORT_STACK_SIZE 4096

// Deadlines em microssegundos
#define DEADLINE_FUS_US   5000
#define DEADLINE_CTRL_US  5000
#define DEADLINE_NAV_US   20000
#define DEADLINE_FS_US    10000

// Threshold automático do touch = 2/3 do valor sem toque
#define TOUCH_THRESHOLD_NUM 2
#define TOUCH_THRESHOLD_DEN 3

#ifndef configUSE_TIME_SLICING
#define configUSE_TIME_SLICING 0
#endif

// ==========================================================
// HANDLES DAS TASKS
// ==========================================================

// Handles usados para sinalizacao direta entre tasks e pela ISR.
static TaskHandle_t h_fus_imu  = NULL;
static TaskHandle_t h_ctrl_att = NULL;
static TaskHandle_t h_nav_plan = NULL;
static TaskHandle_t h_fs_task  = NULL;

// ==========================================================
// TOUCH SENSOR
// ==========================================================

static touch_sensor_handle_t touch_sensor = NULL;

static touch_channel_handle_t touch_nav = NULL;
static touch_channel_handle_t touch_tel = NULL;
static touch_channel_handle_t touch_fs  = NULL;

// ==========================================================
// ESTADO SIMULADO DO DRONE
// ==========================================================

// Estado minimo do drone usado apenas para simular dados de atitude.
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} drone_state_t;

static drone_state_t drone_state = {
    .roll = 0.0f,
    .pitch = 0.0f,
    .yaw = 0.0f
};

// ==========================================================
// EVENTOS NAV / TELEMETRIA
// ==========================================================

typedef enum
{
    EV_NAV = 1,
    EV_TELEMETRIA = 2
} nav_event_type_t;

// Cada evento enviado para NAV_PLAN carrega o tipo e o instante do toque.
// O timestamp permite medir a latencia entre evento e inicio da task.
typedef struct
{
    nav_event_type_t tipo;
    int64_t timestamp_evento;
} nav_event_t;

// Fila compartilhada pelos eventos de navegacao e telemetria.
static QueueHandle_t q_nav = NULL;

// ==========================================================
// FAIL-SAFE
// ==========================================================

// Variaveis compartilhadas entre a ISR do Touch D e a FS_TASK.
// volatile evita que o compilador assuma que os valores nao mudam externamente.
static volatile uint32_t fs_timestamp_evento = 0;
static volatile uint32_t fs_ultimo_toque_us = 0;

#define FS_DEBOUNCE_US 150000   // 150 ms
// ==========================================================
// ESTRUTURA DE MÉTRICAS
// ==========================================================

// Estrutura generica de instrumentacao temporal para cada task.
// Guarda contadores, acumuladores e piores casos observados.
typedef struct
{
    uint32_t execucoes;
    uint32_t misses;

    uint64_t total_exec_us;
    uint64_t total_latencia_us;

    uint32_t wcet_us;
    uint32_t latencia_max_us;
    uint32_t resposta_max_us;
    uint32_t jitter_max_us;

    uint64_t ultimo_evento_us;
    uint64_t ultimo_inicio_us;
    uint64_t ultimo_fim_us;

} task_metrics_t;

static task_metrics_t m_fus;
static task_metrics_t m_ctrl;
static task_metrics_t m_nav;
static task_metrics_t m_fs;

// Protege as estruturas de metricas contra acesso concorrente.
static portMUX_TYPE metrics_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile bool metrics_enabled = false;

static int64_t scenario_start_us = 0;

static uint32_t scenario_number = 1;

// ==========================================================
// FUNÇÕES AUXILIARES
// ==========================================================

// Simula carga computacional mantendo a CPU ocupada por um tempo em us.
// Foi usada para aproximar os tempos C definidos no enunciado.
static void busy_wait_us(uint32_t tempo_us)
{
    int64_t inicio = esp_timer_get_time();

    while ((esp_timer_get_time() - inicio) < tempo_us)
    {
        __asm__ __volatile__("nop");
    }
}

// Diferenca absoluta entre dois timestamps; usada no calculo do jitter.
static uint32_t abs_diff_us(int64_t a, int64_t b)
{
    int64_t d = a - b;

    if (d < 0)
        d = -d;

    return (uint32_t)d;
}

// ==========================================================
// RESET DAS MÉTRICAS
// ==========================================================

// Zera todas as estatisticas antes do inicio de um novo cenario.
static void reset_metrics(void)
{
    // A secao critica garante que nenhuma task altere as metricas durante o reset.
    portENTER_CRITICAL(&metrics_mux);

    memset(&m_fus, 0, sizeof(m_fus));
    memset(&m_ctrl, 0, sizeof(m_ctrl));
    memset(&m_nav, 0, sizeof(m_nav));
    memset(&m_fs, 0, sizeof(m_fs));

    scenario_start_us = esp_timer_get_time();

    portEXIT_CRITICAL(&metrics_mux);
}

// ==========================================================
// REGISTRO FUS_IMU
// ==========================================================

// Atualiza as metricas da FUS_IMU e verifica o deadline de 5 ms.
static void update_fus_metrics(
    uint32_t exec_us,
    uint32_t jitter_us,
    uint32_t response_us,
    uint64_t evento,
    uint64_t inicio,
    uint64_t fim)
{
    if (!metrics_enabled)
        return;

    portENTER_CRITICAL(&metrics_mux);

    m_fus.execucoes++;
    m_fus.total_exec_us += exec_us;

    if (exec_us > m_fus.wcet_us)
        m_fus.wcet_us = exec_us;

    if (jitter_us > m_fus.jitter_max_us)
        m_fus.jitter_max_us = jitter_us;

    if (response_us > m_fus.resposta_max_us)
        m_fus.resposta_max_us = response_us;

    if (response_us > DEADLINE_FUS_US)
        m_fus.misses++;

    m_fus.ultimo_evento_us = evento;
    m_fus.ultimo_inicio_us = inicio;
    m_fus.ultimo_fim_us = fim;

    portEXIT_CRITICAL(&metrics_mux);
}

// ==========================================================
// REGISTRO CTRL_ATT
// ==========================================================

// Atualiza as metricas da CTRL_ATT e verifica o deadline de 5 ms.
static void update_ctrl_metrics(
    uint32_t exec_us,
    uint32_t latency_us,
    uint32_t response_us,
    uint64_t evento,
    uint64_t inicio,
    uint64_t fim)
{
    if (!metrics_enabled)
        return;

    portENTER_CRITICAL(&metrics_mux);

    m_ctrl.execucoes++;
    m_ctrl.total_exec_us += exec_us;
    m_ctrl.total_latencia_us += latency_us;

    if (exec_us > m_ctrl.wcet_us)
        m_ctrl.wcet_us = exec_us;

    if (latency_us > m_ctrl.latencia_max_us)
        m_ctrl.latencia_max_us = latency_us;

    if (response_us > m_ctrl.resposta_max_us)
        m_ctrl.resposta_max_us = response_us;

    if (response_us > DEADLINE_CTRL_US)
        m_ctrl.misses++;

    m_ctrl.ultimo_evento_us = evento;
    m_ctrl.ultimo_inicio_us = inicio;
    m_ctrl.ultimo_fim_us = fim;

    portEXIT_CRITICAL(&metrics_mux);
}

// ==========================================================
// REGISTRO NAV_PLAN
// ==========================================================

// Atualiza as metricas da NAV_PLAN/telemetria e verifica o deadline de 20 ms.
static void update_nav_metrics(
    uint32_t exec_us,
    uint32_t latency_us,
    uint32_t response_us,
    uint64_t evento,
    uint64_t inicio,
    uint64_t fim)
{
    if (!metrics_enabled)
        return;

    portENTER_CRITICAL(&metrics_mux);

    m_nav.execucoes++;
    m_nav.total_exec_us += exec_us;
    m_nav.total_latencia_us += latency_us;

    if (exec_us > m_nav.wcet_us)
        m_nav.wcet_us = exec_us;

    if (latency_us > m_nav.latencia_max_us)
        m_nav.latencia_max_us = latency_us;

    if (response_us > m_nav.resposta_max_us)
        m_nav.resposta_max_us = response_us;

    if (response_us > DEADLINE_NAV_US)
        m_nav.misses++;

    m_nav.ultimo_evento_us = evento;
    m_nav.ultimo_inicio_us = inicio;
    m_nav.ultimo_fim_us = fim;

    portEXIT_CRITICAL(&metrics_mux);
}

// ==========================================================
// REGISTRO FAIL-SAFE
// ==========================================================

// Atualiza as metricas do fail-safe e verifica o deadline de 10 ms.
static void update_fs_metrics(
    uint32_t exec_us,
    uint32_t latency_us,
    uint32_t response_us,
    uint64_t evento,
    uint64_t inicio,
    uint64_t fim)
{
    if (!metrics_enabled)
        return;

    portENTER_CRITICAL(&metrics_mux);

    m_fs.execucoes++;
    m_fs.total_exec_us += exec_us;
    m_fs.total_latencia_us += latency_us;

    if (exec_us > m_fs.wcet_us)
        m_fs.wcet_us = exec_us;

    if (latency_us > m_fs.latencia_max_us)
        m_fs.latencia_max_us = latency_us;

    if (response_us > m_fs.resposta_max_us)
        m_fs.resposta_max_us = response_us;

    if (response_us > DEADLINE_FS_US)
        m_fs.misses++;

    m_fs.ultimo_evento_us = evento;
    m_fs.ultimo_inicio_us = inicio;
    m_fs.ultimo_fim_us = fim;

    portEXIT_CRITICAL(&metrics_mux);
}

// ==========================================================
// TOUCH B / C
// ==========================================================

// Callback de ativacao dos Touch B e C.
// Em vez de executar processamento pesado aqui, apenas empacota o evento
// e o envia para a fila da NAV_PLAN.
static bool touch_active_callback(
    touch_sensor_handle_t sens_handle,
    const touch_active_event_data_t *event,
    void *user_ctx)
{
    nav_event_t ev;

    if (event->chan_id == TOUCH_NAV_CH)
    {
        ev.tipo = EV_NAV;
        ev.timestamp_evento = esp_timer_get_time();

        xQueueSend(q_nav, &ev, 0);
    }
    else if (event->chan_id == TOUCH_TEL_CH)
    {
        ev.tipo = EV_TELEMETRIA;
        ev.timestamp_evento = esp_timer_get_time();

        xQueueSend(q_nav, &ev, 0);
    }

    return false;
}

// ==========================================================
// TOUCH LIBERADO
// ==========================================================

// Callback de liberacao do touch.
// Nao ha processamento necessario neste experimento.
static bool touch_inactive_callback(
    touch_sensor_handle_t sens_handle,
    const touch_inactive_event_data_t *event,
    void *user_ctx)
{
    return false;
}

// ==========================================================
// TOUCH D - HARDWARE ISR
// ==========================================================

// Callback de hardware do Touch D.
// Roda em contexto de interrupcao e, por isso, faz apenas o minimo necessario:
// registra o timestamp, aplica debounce e acorda a FS_TASK por notificacao.
static bool IRAM_ATTR touch_hw_callback(
    touch_sensor_handle_t sens_handle,
    const touch_hw_active_event_data_t *event,
    void *user_ctx)
{
    BaseType_t tarefa_prioritaria_acordada = pdFALSE;

    if ((event->active_mask & (1UL << TOUCH_FS_CH)) != 0)
    {
        uint32_t agora =
            (uint32_t)esp_timer_get_time();

        /*
         * Debounce temporal:
         * considera um novo evento somente se passaram
         * pelo menos 150 ms desde o anterior.
         */
        // Ignora pulsos repetidos dentro da janela de debounce.
        if ((uint32_t)(agora - fs_ultimo_toque_us) >= FS_DEBOUNCE_US)
        {
            fs_ultimo_toque_us = agora;
            fs_timestamp_evento = agora;

            if (h_fs_task != NULL)
            {
                // Notificacao direta e leve para acordar imediatamente a task de seguranca.
                vTaskNotifyGiveFromISR(
                    h_fs_task,
                    &tarefa_prioritaria_acordada
                );
            }
        }
    }

    return (tarefa_prioritaria_acordada == pdTRUE);
}

// ==========================================================
// TASK FUS_IMU
// T = 5 ms
// D = 5 ms
// C ≈ 1 ms
// ==========================================================

// Tarefa periodica de maior frequencia do sistema.
// vTaskDelayUntil() mantem a referencia temporal absoluta do periodo,
// reduzindo deriva acumulada entre ativacoes.
static void task_fus_imu(void *arg)
{
    TickType_t ultimo_instante = xTaskGetTickCount();

    const TickType_t periodo =
        pdMS_TO_TICKS(FUS_IMU_PERIOD_MS);

    int64_t release_esperado_us = 0;
    bool primeira_execucao = true;

    while (1)
    {
        // Aguarda a proxima liberacao periodica de 5 ms.
        vTaskDelayUntil(
            &ultimo_instante,
            periodo
        );

        int64_t inicio = esp_timer_get_time();

        // Na primeira ativacao, o proprio inicio vira a referencia temporal.
        // Nas seguintes, o release esperado avanca exatamente 5 ms.
        if (primeira_execucao)
        {
            release_esperado_us = inicio;
            primeira_execucao = false;
        }
        else
        {
            release_esperado_us +=
                (FUS_IMU_PERIOD_MS * 1000);
        }

        // Jitter = desvio entre o instante real de inicio e o release esperado.
        uint32_t jitter =
            abs_diff_us(
                inicio,
                release_esperado_us
            );

        // Simulacao simples de uma etapa de fusao/atualizacao da atitude.
        drone_state.roll *= 0.98f;
        drone_state.pitch *= 0.98f;
        drone_state.yaw *= 0.98f;

        drone_state.roll += 0.10f;
        drone_state.yaw += 0.05f;

        busy_wait_us(1000);

        int64_t fim = esp_timer_get_time();

        uint32_t exec_us =
            (uint32_t)(fim - inicio);

        // Tempo de resposta medido desde o release esperado ate o termino.
        // Isso permite contabilizar atraso de escalonamento como parte do deadline.
        uint32_t response_us =
            (fim > release_esperado_us)
            ? (uint32_t)(fim - release_esperado_us)
            : exec_us;

        update_fus_metrics(
            exec_us,
            jitter,
            response_us,
            release_esperado_us,
            inicio,
            fim
        );

        // A CTRL_ATT e liberada somente depois que a FUS_IMU termina.
        if (h_ctrl_att != NULL)
        {
            uint32_t timestamp_ctrl =
                (uint32_t)fim;

            // O timestamp da conclusao da FUS_IMU segue junto na notificacao,
            // permitindo medir a latencia ate o inicio da CTRL_ATT.
            xTaskNotify(
                h_ctrl_att,
                timestamp_ctrl,
                eSetValueWithOverwrite
            );
        }
    }
}

// ==========================================================
// TASK CTRL_ATT
// D = 5 ms após FUS_IMU
// C ≈ 0.8 ms
// ==========================================================

// Tarefa encadeada de controle de atitude.
// Fica bloqueada esperando notificacao da FUS_IMU, sem polling.
static void task_ctrl_att(void *arg)
{
    uint32_t timestamp_evento;

    while (1)
    {
        // Espera indefinidamente a conclusao da ultima FUS_IMU.
        xTaskNotifyWait(
            0x00,
            UINT32_MAX,
            &timestamp_evento,
            portMAX_DELAY
        );

        uint32_t inicio =
            (uint32_t)esp_timer_get_time();

        // Latencia entre o fim da FUS_IMU e o inicio efetivo do controle.
        uint32_t latency_us =
            inicio - timestamp_evento;

        // PID / atuação simulada
        busy_wait_us(800);

        uint32_t fim =
            (uint32_t)esp_timer_get_time();

        uint32_t exec_us =
            fim - inicio;

        // Tempo de resposta medido desde o release esperado ate o termino.
        // Isso permite contabilizar atraso de escalonamento como parte do deadline.
        uint32_t response_us =
            fim - timestamp_evento;

        update_ctrl_metrics(
            exec_us,
            latency_us,
            response_us,
            timestamp_evento,
            inicio,
            fim
        );
    }
}

// ==========================================================
// TASK NAV_PLAN
// D = 20 ms
// ==========================================================

// Tarefa event-driven para navegacao e telemetria.
// Bloqueia na fila e so consome CPU quando B ou C gera um evento.
static void task_nav_plan(void *arg)
{
    nav_event_t evento;

    while (1)
    {
        // portMAX_DELAY evita polling e deixa a task bloqueada sem gastar CPU.
        if (xQueueReceive(
                q_nav,
                &evento,
                portMAX_DELAY) == pdTRUE)
        {
            int64_t inicio =
                esp_timer_get_time();

            uint32_t latency_us =
                (uint32_t)
                (inicio - evento.timestamp_evento);

            // O evento B simula planejamento de rota, com carga maior.
            if (evento.tipo == EV_NAV)
            {
                // Planejamento de rota
                busy_wait_us(3500);
            }
            // O evento C simula leitura/publicacao de telemetria.
            else if (evento.tipo == EV_TELEMETRIA)
            {
                // Telemetria simulada
                volatile float r = drone_state.roll;
                volatile float p = drone_state.pitch;
                volatile float y = drone_state.yaw;

                (void)r;
                (void)p;
                (void)y;

                busy_wait_us(500);
            }

            int64_t fim =
                esp_timer_get_time();

            uint32_t exec_us =
                (uint32_t)(fim - inicio);

            // Tempo de resposta medido desde o release esperado ate o termino.
        // Isso permite contabilizar atraso de escalonamento como parte do deadline.
        uint32_t response_us =
                (uint32_t)
                (fim - evento.timestamp_evento);

            update_nav_metrics(
                exec_us,
                latency_us,
                response_us,
                evento.timestamp_evento,
                inicio,
                fim
            );
        }
    }
}

// ==========================================================
// TASK FAIL-SAFE
// Touch D -> ISR -> Task
// D = 10 ms
// ==========================================================

// Tarefa hard real-time de emergencia.
// E acordada diretamente pela ISR do Touch D e simula uma acao de pouso/hover seguro.
static void task_fail_safe(void *arg)
{
    while (1)
    {
        // Bloqueia ate a ISR sinalizar um novo evento de fail-safe.
        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        uint32_t evento =
            fs_timestamp_evento;

        uint32_t inicio =
            (uint32_t)esp_timer_get_time();

        // Latencia ISR -> task: tempo entre o toque D e o inicio do fail-safe.
        uint32_t latency_us =
            inicio - evento;

        // Pouso / hover seguro simulado
        busy_wait_us(900);

        uint32_t fim =
            (uint32_t)esp_timer_get_time();

        uint32_t exec_us =
            fim - inicio;

        // Tempo de resposta medido desde o release esperado ate o termino.
        // Isso permite contabilizar atraso de escalonamento como parte do deadline.
        uint32_t response_us =
            fim - evento;

        update_fs_metrics(
            exec_us,
            latency_us,
            response_us,
            evento,
            inicio,
            fim
        );
    }
}

// ==========================================================
// RELATÓRIO DO CENÁRIO
// ==========================================================

// Tarefa de baixa prioridade que fecha cada janela de 30 s,
// copia as metricas e imprime um resumo para o relatorio experimental.
static void task_report(void *arg)
{
    while (1)
    {
        vTaskDelay(
            pdMS_TO_TICKS(TEST_DURATION_MS)
        );

        // Congela a coleta enquanto o relatorio do cenario e montado.
        metrics_enabled = false;

        task_metrics_t fus;
        task_metrics_t ctrl;
        task_metrics_t nav;
        task_metrics_t fs;

        int64_t inicio_cenario;
        int64_t fim_cenario =
            esp_timer_get_time();

        portENTER_CRITICAL(&metrics_mux);

        fus = m_fus;
        ctrl = m_ctrl;
        nav = m_nav;
        fs = m_fs;

        inicio_cenario =
            scenario_start_us;

        portEXIT_CRITICAL(&metrics_mux);

        double duracao_s =
            (fim_cenario - inicio_cenario) /
            1000000.0;

        uint64_t busy_total_us =
            fus.total_exec_us +
            ctrl.total_exec_us +
            nav.total_exec_us +
            fs.total_exec_us;

        // Ocupacao estimada considera apenas o tempo observado das quatro tasks do drone.
        // Nao representa a utilizacao total da CPU pelo ESP-IDF.
        double ocupacao =
            100.0 *
            ((double)busy_total_us /
            (double)(fim_cenario - inicio_cenario));

        uint32_t hard_exec =
            fus.execucoes +
            ctrl.execucoes +
            fs.execucoes;

        uint32_t hard_misses =
            fus.misses +
            ctrl.misses +
            fs.misses;

        double hard_miss_percent = 0.0;

        if (hard_exec > 0)
        {
            hard_miss_percent =
                100.0 *
                ((double)hard_misses /
                (double)hard_exec);
        }

        double soft_miss_percent = 0.0;

        if (nav.execucoes > 0)
        {
            soft_miss_percent =
                100.0 *
                ((double)nav.misses /
                (double)nav.execucoes);
        }

        double nav_latency_avg = 0.0;

        if (nav.execucoes > 0)
        {
            nav_latency_avg =
                (double)nav.total_latencia_us /
                nav.execucoes;
        }

        double fs_latency_avg = 0.0;

        if (fs.execucoes > 0)
        {
            fs_latency_avg =
                (double)fs.total_latencia_us /
                fs.execucoes;
        }

        printf("\n\n");
        printf("============================================================\n");
        printf("              RESULTADO DO CENARIO %lu\n",
               (unsigned long)scenario_number);
        printf("============================================================\n");

        printf("Politica       : %s\n", POLICY_NAME);
        printf("CPU            : %d MHz\n",
               CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
        printf("Core das tasks : 0\n");

        printf("Preempcao      : %s\n",
               configUSE_PREEMPTION ? "ON" : "OFF");

        printf("Time Slicing   : %s\n",
               configUSE_TIME_SLICING ? "ON" : "OFF");

        printf("Duracao        : %.3f s\n",
               duracao_s);

        printf("------------------------------------------------------------\n");

        printf("FUS_IMU\n");
        printf("  Execucoes       : %lu\n",
               (unsigned long)fus.execucoes);
        printf("  WCET            : %lu us\n",
               (unsigned long)fus.wcet_us);
        printf("  Jitter max      : %lu us\n",
               (unsigned long)fus.jitter_max_us);
        printf("  Resposta max    : %lu us\n",
               (unsigned long)fus.resposta_max_us);
        printf("  Deadline misses : %lu\n",
               (unsigned long)fus.misses);

        printf("  Ultimo evento   : %llu us\n",
               (unsigned long long)fus.ultimo_evento_us);
        printf("  Ultimo inicio   : %llu us\n",
               (unsigned long long)fus.ultimo_inicio_us);
        printf("  Ultimo fim      : %llu us\n",
               (unsigned long long)fus.ultimo_fim_us);

        printf("------------------------------------------------------------\n");

        printf("CTRL_ATT\n");
        printf("  Execucoes       : %lu\n",
               (unsigned long)ctrl.execucoes);
        printf("  WCET            : %lu us\n",
               (unsigned long)ctrl.wcet_us);
        printf("  Latencia max    : %lu us\n",
               (unsigned long)ctrl.latencia_max_us);
        printf("  Resposta max    : %lu us\n",
               (unsigned long)ctrl.resposta_max_us);
        printf("  Deadline misses : %lu\n",
               (unsigned long)ctrl.misses);

        printf("  Ultimo evento   : %llu us\n",
               (unsigned long long)ctrl.ultimo_evento_us);
        printf("  Ultimo inicio   : %llu us\n",
               (unsigned long long)ctrl.ultimo_inicio_us);
        printf("  Ultimo fim      : %llu us\n",
               (unsigned long long)ctrl.ultimo_fim_us);

        printf("------------------------------------------------------------\n");

        printf("NAV_PLAN / TELEMETRIA\n");
        printf("  Eventos         : %lu\n",
               (unsigned long)nav.execucoes);
        printf("  WCET            : %lu us\n",
               (unsigned long)nav.wcet_us);
        printf("  Latencia media  : %.2f us\n",
               nav_latency_avg);
        printf("  Latencia max    : %lu us\n",
               (unsigned long)nav.latencia_max_us);
        printf("  Resposta max    : %lu us\n",
               (unsigned long)nav.resposta_max_us);
        printf("  Deadline misses : %lu\n",
               (unsigned long)nav.misses);

        printf("  Ultimo evento   : %llu us\n",
               (unsigned long long)nav.ultimo_evento_us);
        printf("  Ultimo inicio   : %llu us\n",
               (unsigned long long)nav.ultimo_inicio_us);
        printf("  Ultimo fim      : %llu us\n",
               (unsigned long long)nav.ultimo_fim_us);

        printf("------------------------------------------------------------\n");

        printf("FS_TASK\n");
        printf("  Eventos         : %lu\n",
               (unsigned long)fs.execucoes);
        printf("  WCET            : %lu us\n",
               (unsigned long)fs.wcet_us);
        printf("  Latencia media  : %.2f us\n",
               fs_latency_avg);
        printf("  Latencia max    : %lu us\n",
               (unsigned long)fs.latencia_max_us);
        printf("  Resposta max    : %lu us\n",
               (unsigned long)fs.resposta_max_us);
        printf("  Deadline misses : %lu\n",
               (unsigned long)fs.misses);

        printf("  Ultimo evento   : %llu us\n",
               (unsigned long long)fs.ultimo_evento_us);
        printf("  Ultimo inicio   : %llu us\n",
               (unsigned long long)fs.ultimo_inicio_us);
        printf("  Ultimo fim      : %llu us\n",
               (unsigned long long)fs.ultimo_fim_us);

        printf("------------------------------------------------------------\n");

        printf("RESUMO DO CENARIO\n");

        printf("  Hard misses     : %lu / %lu (%.3f%%)\n",
               (unsigned long)hard_misses,
               (unsigned long)hard_exec,
               hard_miss_percent);

        printf("  Soft misses     : %lu / %lu (%.3f%%)\n",
               (unsigned long)nav.misses,
               (unsigned long)nav.execucoes,
               soft_miss_percent);

        printf("  Lat Touch NAV   : max %lu us\n",
               (unsigned long)nav.latencia_max_us);

        printf("  Lat Touch FS    : max %lu us\n",
               (unsigned long)fs.latencia_max_us);

        printf("  Ocupacao CPU estimada (tasks drone): %.2f%%\n",
               ocupacao);

        printf("============================================================\n\n");

        scenario_number++;

        // Pausa pequena só para separar os relatórios
        vTaskDelay(pdMS_TO_TICKS(500));

        reset_metrics();

        metrics_enabled = true;

        printf("\nNOVO CENARIO INICIADO - %lu\n",
               (unsigned long)scenario_number);

        printf("Durante 30 s, acione B, C e D varias vezes.\n\n");
    }
}

// ==========================================================
// INICIALIZAÇÃO DO TOUCH
// ==========================================================

// Configura o periferico touch, mede a linha de base sem toque,
// calcula thresholds automaticos e registra os callbacks dos tres canais.
static void inicializar_touch(void)
{
    ESP_LOGI(
        TAG,
        "Inicializando Touch Sensor ESP-IDF 6.1"
    );

    ESP_LOGW(
        TAG,
        "CALIBRANDO TOUCH - NAO TOQUE NOS PINOS"
    );

    touch_sensor_sample_config_t
        sample_cfg[TOUCH_SAMPLE_CFG_NUM] = {

        TOUCH_SENSOR_V1_DEFAULT_SAMPLE_CONFIG(
            1.0,
            TOUCH_VOLT_LIM_L_0V5,
            TOUCH_VOLT_LIM_H_1V7
        )
    };

    touch_sensor_config_t sensor_cfg =
        TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(
            1,
            sample_cfg
        );

    ESP_ERROR_CHECK(
        touch_sensor_new_controller(
            &sensor_cfg,
            &touch_sensor
        )
    );

    touch_channel_config_t channel_cfg = {
        .abs_active_thresh = {1000},
        .charge_speed =
            TOUCH_CHARGE_SPEED_7,
        .init_charge_volt =
            TOUCH_INIT_CHARGE_VOLT_DEFAULT,
        .group =
            TOUCH_CHAN_TRIG_GROUP_BOTH
    };

    ESP_ERROR_CHECK(
        touch_sensor_new_channel(
            touch_sensor,
            TOUCH_NAV_CH,
            &channel_cfg,
            &touch_nav
        )
    );

    ESP_ERROR_CHECK(
        touch_sensor_new_channel(
            touch_sensor,
            TOUCH_TEL_CH,
            &channel_cfg,
            &touch_tel
        )
    );

    ESP_ERROR_CHECK(
        touch_sensor_new_channel(
            touch_sensor,
            TOUCH_FS_CH,
            &channel_cfg,
            &touch_fs
        )
    );

    touch_sensor_filter_config_t filter_cfg =
        TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();

    ESP_ERROR_CHECK(
        touch_sensor_config_filter(
            touch_sensor,
            &filter_cfg
        )
    );

    ESP_ERROR_CHECK(
        touch_sensor_enable(
            touch_sensor
        )
    );

    ESP_ERROR_CHECK(
        touch_sensor_start_continuous_scanning(
            touch_sensor
        )
    );

    // Aguarda estabilizacao do filtro antes de medir a linha de base.
    vTaskDelay(pdMS_TO_TICKS(500));

    // Leituras de baseline usadas para criar thresholds proporcionais
    // ao valor real de cada eletrodo na placa.
    uint32_t valor_nav = 0;
    uint32_t valor_tel = 0;
    uint32_t valor_fs = 0;

    ESP_ERROR_CHECK(
        touch_channel_read_data(
            touch_nav,
            TOUCH_CHAN_DATA_TYPE_SMOOTH,
            &valor_nav
        )
    );

    ESP_ERROR_CHECK(
        touch_channel_read_data(
            touch_tel,
            TOUCH_CHAN_DATA_TYPE_SMOOTH,
            &valor_tel
        )
    );

    ESP_ERROR_CHECK(
        touch_channel_read_data(
            touch_fs,
            TOUCH_CHAN_DATA_TYPE_SMOOTH,
            &valor_fs
        )
    );

    ESP_LOGI(
        TAG,
        "Touch base | B=%lu C=%lu D=%lu",
        (unsigned long)valor_nav,
        (unsigned long)valor_tel,
        (unsigned long)valor_fs
    );

    ESP_ERROR_CHECK(
        touch_sensor_stop_continuous_scanning(
            touch_sensor
        )
    );

    ESP_ERROR_CHECK(
        touch_sensor_disable(
            touch_sensor
        )
    );

    touch_channel_config_t cfg_nav = {
        .abs_active_thresh = {
            valor_nav *
            TOUCH_THRESHOLD_NUM /
            TOUCH_THRESHOLD_DEN
        },
        .charge_speed =
            TOUCH_CHARGE_SPEED_7,
        .init_charge_volt =
            TOUCH_INIT_CHARGE_VOLT_DEFAULT,
        .group =
            TOUCH_CHAN_TRIG_GROUP_BOTH
    };

    touch_channel_config_t cfg_tel = {
        .abs_active_thresh = {
            valor_tel *
            TOUCH_THRESHOLD_NUM /
            TOUCH_THRESHOLD_DEN
        },
        .charge_speed =
            TOUCH_CHARGE_SPEED_7,
        .init_charge_volt =
            TOUCH_INIT_CHARGE_VOLT_DEFAULT,
        .group =
            TOUCH_CHAN_TRIG_GROUP_BOTH
    };

    touch_channel_config_t cfg_fs = {
        .abs_active_thresh = {
            valor_fs *
            TOUCH_THRESHOLD_NUM /
            TOUCH_THRESHOLD_DEN
        },
        .charge_speed =
            TOUCH_CHARGE_SPEED_7,
        .init_charge_volt =
            TOUCH_INIT_CHARGE_VOLT_DEFAULT,
        .group =
            TOUCH_CHAN_TRIG_GROUP_BOTH
    };

    ESP_ERROR_CHECK(
        touch_sensor_reconfig_channel(
            touch_nav,
            &cfg_nav
        )
    );

    ESP_ERROR_CHECK(
        touch_sensor_reconfig_channel(
            touch_tel,
            &cfg_tel
        )
    );

    ESP_ERROR_CHECK(
        touch_sensor_reconfig_channel(
            touch_fs,
            &cfg_fs
        )
    );

    ESP_LOGI(
        TAG,
        "Threshold | B=%lu C=%lu D=%lu",
        (unsigned long)
            cfg_nav.abs_active_thresh[0],

        (unsigned long)
            cfg_tel.abs_active_thresh[0],

        (unsigned long)
            cfg_fs.abs_active_thresh[0]
    );

    // Associa callbacks de software (B/C) e de hardware/ISR (D).
    touch_event_callbacks_t callbacks = {
        .on_active =
            touch_active_callback,

        .on_inactive =
            touch_inactive_callback,

        .on_hw_active =
            touch_hw_callback
    };

    ESP_ERROR_CHECK(
        touch_sensor_register_callbacks(
            touch_sensor,
            &callbacks,
            NULL
        )
    );

    ESP_ERROR_CHECK(
        touch_sensor_enable(
            touch_sensor
        )
    );

    ESP_ERROR_CHECK(
        touch_sensor_start_continuous_scanning(
            touch_sensor
        )
    );

    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, " TOUCH SENSOR PRONTO");
    ESP_LOGI(TAG, " B: GPIO27 -> NAV");
    ESP_LOGI(TAG, " C: GPIO33 -> TELEMETRIA");
    ESP_LOGI(TAG, " D: GPIO32 -> FAIL-SAFE ISR");
    ESP_LOGI(TAG, "=================================");
}

// ==========================================================
// APP_MAIN
// ==========================================================

// Ponto de entrada da aplicacao.
// Cria recursos do FreeRTOS, fixa as tasks no core 0, inicializa touch
// e somente depois inicia a janela de medicao.
void app_main(void)
{
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "   PROJETO RTS - DRONE");
    ESP_LOGI(TAG, "=================================");

    // Fila com capacidade para oito eventos touch pendentes.
    q_nav = xQueueCreate(
        8,
        sizeof(nav_event_t)
    );

    if (q_nav == NULL)
    {
        ESP_LOGE(
            TAG,
            "Erro ao criar q_nav"
        );

        return;
    }

    // ======================================================
    // TODAS AS TASKS DO DRONE NO CORE 0
    // O pinning elimina migracao entre cores e deixa a comparacao mais controlada.
    // ======================================================

    xTaskCreatePinnedToCore(
        task_fail_safe,
        "FS_TASK",
        TASK_STACK_SIZE,
        NULL,
        PRIO_FS_TASK,
        &h_fs_task,
        0
    );

    xTaskCreatePinnedToCore(
        task_fus_imu,
        "FUS_IMU",
        TASK_STACK_SIZE,
        NULL,
        PRIO_FUS_IMU,
        &h_fus_imu,
        0
    );

    xTaskCreatePinnedToCore(
        task_ctrl_att,
        "CTRL_ATT",
        TASK_STACK_SIZE,
        NULL,
        PRIO_CTRL_ATT,
        &h_ctrl_att,
        0
    );

    xTaskCreatePinnedToCore(
        task_nav_plan,
        "NAV_PLAN",
        TASK_STACK_SIZE,
        NULL,
        PRIO_NAV_PLAN,
        &h_nav_plan,
        0
    );

    ESP_LOGI(
        TAG,
        "Tasks do drone criadas no Core 0"
    );

    ESP_LOGI(
        TAG,
        "Politica: %s",
        POLICY_NAME
    );

    ESP_LOGI(
        TAG,
        "Prioridades: FS=%d FUS=%d CTRL=%d NAV=%d",
        PRIO_FS_TASK,
        PRIO_FUS_IMU,
        PRIO_CTRL_ATT,
        PRIO_NAV_PLAN
    );

    // Inicializa e calibra os sensores antes de iniciar as metricas.
    inicializar_touch();

    // ======================================================
    // COMEÇA O EXPERIMENTO SOMENTE DEPOIS DA CALIBRAÇÃO
    // ======================================================

    reset_metrics();

    // A partir daqui as execucoes passam a ser contabilizadas.
    metrics_enabled = true;

    xTaskCreatePinnedToCore(
        task_report,
        "REPORT",
        REPORT_STACK_SIZE,
        NULL,
        PRIO_REPORT,
        NULL,
        0
    );

    printf("\n");
    printf("============================================================\n");
    printf("              CENARIO 1 INICIADO\n");
    printf("============================================================\n");
    printf("Politica   : %s\n", POLICY_NAME);
    printf("CPU        : %d MHz\n",
           CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    printf("Core tasks : 0\n");
    printf("Duracao    : 30 segundos\n");
    printf("\n");
    printf("Durante os 30 segundos, acione B, C e D varias vezes.\n");
    printf("Ao final o resumo sera impresso automaticamente.\n");
    printf("============================================================\n\n");
}