#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "string.h"
#include "rtos_monitor.h"
#include "log_vt100.h"
#include "rack_inteligente_parametros.h"

#if ( ENABLE_RTOS_ANALYSIS == 1 )
#include "hardware/clocks.h"
#include "hardware/structs/scb.h"
#endif

#if ( ENABLE_RTOS_ANALYSIS == 1 )
#include "rtos_trace.h"
#include "wcet_probe.h"
#endif

#if ( ENABLE_RTOS_ANALYSIS == 1 )
static void vPrintRuntimeStats(void)
{
#if ( configGENERATE_RUN_TIME_STATS == 1 ) && ( configUSE_STATS_FORMATTING_FUNCTIONS == 1 )
    static char statsBuffer[1024];

    memset(statsBuffer, 0, sizeof(statsBuffer));
    vTaskGetRunTimeStats(statsBuffer);

    LOG_INFO("==================== Run Time Stats (CPU) ====================");

    char *saveptr = NULL;
    char *line = strtok_r(statsBuffer, "\r\n", &saveptr);
    while (line != NULL)
    {
        if (line[0] != '\0')
        {
            LOG_INFO("%s", line);
        }
        line = strtok_r(NULL, "\r\n", &saveptr);
    }

    LOG_INFO("==================== End Run Time Stats ====================");
#endif
}
#endif


#if ENABLE_STACK_WATERMARK

/**
 * @brief Calcula o tamanho total da pilha de uma tarefa em words.
 *
 * Utiliza os ponteiros pxEndOfStack e pxStackBase fornecidos pelo
 * TaskStatus_t quando configRECORD_STACK_HIGH_ADDRESS está habilitado.
 *
 * @param pxStatus Ponteiro para estrutura TaskStatus_t da tarefa.
 * @return Tamanho total da pilha em words, ou 0 se não disponível.
 */
static inline size_t xGetTaskStackSize(const TaskStatus_t *pxStatus)
{
#if ( configRECORD_STACK_HIGH_ADDRESS == 1 )
    if (pxStatus->pxEndOfStack != NULL && pxStatus->pxStackBase != NULL)
    {
        /* Calcula diferença entre topo e base da pilha */
        return (size_t)(pxStatus->pxEndOfStack - pxStatus->pxStackBase);
    }
#endif
    (void)pxStatus;
    return 0;
}

/**
 * @brief Imprime o watermark de pilha de todas as tarefas ativas.
 *
 * Esta função obtém a lista de tarefas do FreeRTOS e imprime:
 * - Tamanho total da pilha de cada tarefa
 * - Watermark (mínimo livre desde criação)
 * - Tamanho usado (total - watermark)
 * - Porcentagem de uso individual
 * - Totalizadores globais
 *
 * @note Quanto maior a porcentagem de uso, mais próximo do estouro.
 *       Valores acima de 80% indicam risco potencial.
 */
static void vPrintStackWatermarks(void)
{
    TaskStatus_t taskStatusArray[RTOS_MONITOR_MAX_TASKS];
    UBaseType_t taskCount;
    UBaseType_t i;

    /* Acumuladores para estatísticas globais */
    size_t totalStackAllocated = 0;
    size_t totalStackUsed = 0;

    /* Obtém snapshot do estado de todas as tarefas */
    taskCount = uxTaskGetSystemState(taskStatusArray, RTOS_MONITOR_MAX_TASKS, NULL);

    if (taskCount == 0)
    {
        LOG_WARN("[Watermark] Nenhuma tarefa encontrada");
        return;
    }

    LOG_INFO("==================== Stack Watermark Report ====================");
    LOG_INFO("%-16s | %8s | %8s | %8s | %5s | %s",
             "Tarefa", "Total", "Livre", "Usado", "Uso%", "Status");
    LOG_INFO("-----------------------------------------------------------------");

    for (i = 0; i < taskCount; i++)
    {
        const TaskStatus_t *pxTask = &taskStatusArray[i];
        size_t stackTotal = xGetTaskStackSize(pxTask);
        size_t watermark = (size_t)pxTask->usStackHighWaterMark;
        size_t stackUsed = 0;
        uint8_t usagePercent = 0;
        const char *status = "N/A";

        /* Calcula uso se tamanho total disponível */
        if (stackTotal > 0)
        {
            stackUsed = stackTotal - watermark;
            usagePercent = (uint8_t)((stackUsed * 100) / stackTotal);

            /* Classifica o risco baseado na porcentagem de uso */
            if (usagePercent >= 90)
            {
                status = "CRITICO";
            }
            else if (usagePercent >= 75)
            {
                status = "ATENCAO";
            }
            else
            {
                status = "OK";
            }

            /* Acumula para totais */
            totalStackAllocated += stackTotal;
            totalStackUsed += stackUsed;
        }
        else
        {
            /* Fallback: classifica pelo watermark absoluto */
            if (watermark < 50)
            {
                status = "CRITICO";
            }
            else if (watermark < 100)
            {
                status = "ATENCAO";
            }
            else
            {
                status = "OK";
            }
        }

        LOG_INFO("%-16s | %8u | %8u | %8u | %4u%% | %s",
                 pxTask->pcTaskName,
                 (unsigned int)(stackTotal * sizeof(StackType_t)),
                 (unsigned int)(watermark * sizeof(StackType_t)),
                 (unsigned int)(stackUsed * sizeof(StackType_t)),
                 (unsigned int)usagePercent,
                 status);

#if ( ENABLE_RTOS_ANALYSIS == 1 )
        LOG_INFO("[MonitorKV] task=%s stack_total=%u stack_free=%u stack_used=%u stack_usage_pct=%u",
                 pxTask->pcTaskName,
                 (unsigned int)(stackTotal * sizeof(StackType_t)),
                 (unsigned int)(watermark * sizeof(StackType_t)),
                 (unsigned int)(stackUsed * sizeof(StackType_t)),
                 (unsigned int)usagePercent);
#endif
    }

    /* Imprime totalizadores */
    LOG_INFO("-----------------------------------------------------------------");

    if (totalStackAllocated > 0)
    {
        uint8_t totalUsagePercent = (uint8_t)((totalStackUsed * 100) / totalStackAllocated);

        LOG_INFO("%-16s | %8u | %8u | %8u | %4u%%",
                 "TOTAL",
                 (unsigned int)(totalStackAllocated * sizeof(StackType_t)),
                 (unsigned int)((totalStackAllocated - totalStackUsed) * sizeof(StackType_t)),
                 (unsigned int)(totalStackUsed * sizeof(StackType_t)),
                 (unsigned int)totalUsagePercent);

        LOG_INFO("Tarefas: %u | Stack total: %u bytes | Uso global: %u%%",
                 (unsigned int)taskCount,
                 (unsigned int)(totalStackAllocated * sizeof(StackType_t)),
                 (unsigned int)totalUsagePercent);
    }
    else
    {
        LOG_WARN("Tamanho de pilha nao disponivel (habilite configRECORD_STACK_HIGH_ADDRESS)");
    }

    LOG_INFO("==================== Fim do Report ====================");
}
#endif /* ENABLE_STACK_WATERMARK */

/**
 * @brief Tarefa de monitoramento do sistema.
 *
 * Monitora o estado do sistema, incluindo uso de memória e
 * estado das tarefas. Útil para debug e diagnóstico.
 * 
 * Quando ENABLE_STACK_WATERMARK está ativo, também imprime
 * o watermark de pilha de todas as tarefas para análise.
 *
 * @param pvParameters Parâmetros da tarefa (não utilizado).
 */
void vRTOSMonitorTask(void *pvParameters)
{
    (void)pvParameters;

#if ( ENABLE_RTOS_ANALYSIS == 1 )
    rtosTraceInit();
#endif

    for (;;)
    {
        /* Monitorar heap livre */
        size_t freeHeap = xPortGetFreeHeapSize();
        size_t minFreeHeap = xPortGetMinimumEverFreeHeapSize();

        /* Log periódico de status */
        LOG_INFO("[Monitor] Heap livre: %u bytes, Minimo: %u bytes",
                 (unsigned int)freeHeap, (unsigned int)minFreeHeap);

#if ( ENABLE_RTOS_ANALYSIS == 1 )
        LOG_INFO("[MonitorKV] heap_free=%u heap_min=%u",
                 (unsigned int)freeHeap, (unsigned int)minFreeHeap);

        {
            const uint32_t clkRefHz = (uint32_t)clock_get_hz(clk_ref);
            const uint32_t clkSysHz = (uint32_t)clock_get_hz(clk_sys);
            const uint32_t clkPeriHz = (uint32_t)clock_get_hz(clk_peri);
            const uint32_t clkUsbHz = (uint32_t)clock_get_hz(clk_usb);
            const uint32_t clkAdcHz = (uint32_t)clock_get_hz(clk_adc);
            const uint32_t clkRtcHz = (uint32_t)clock_get_hz(clk_rtc);
            const uint32_t scbIcsr = scb_hw->icsr;
            LOG_INFO(
                "[HwKV] clk_ref_hz=%u clk_sys_hz=%u clk_peri_hz=%u clk_usb_hz=%u clk_adc_hz=%u clk_rtc_hz=%u scb_icsr=0x%08X "
                "rtos_tick_hz=%u rtos_preempt=%u rtos_tickless=%u rtos_max_prio=%u",
                (unsigned int)clkRefHz,
                (unsigned int)clkSysHz,
                (unsigned int)clkPeriHz,
                (unsigned int)clkUsbHz,
                (unsigned int)clkAdcHz,
                (unsigned int)clkRtcHz,
                (unsigned int)scbIcsr,
                (unsigned int)configTICK_RATE_HZ,
                (unsigned int)configUSE_PREEMPTION,
                (unsigned int)configUSE_TICKLESS_IDLE,
                (unsigned int)configMAX_PRIORITIES);
        }
        vPrintRuntimeStats();

        {
            char traceLine[160];
            while (rtosTracePopLogLine(traceLine, sizeof(traceLine)) != 0)
            {
                LOG_INFO("%s", traceLine);
            }
        }

        {
            char wcetLine[180];
            while (wcetProbePopLogLine(wcetLine, sizeof(wcetLine)) != 0)
            {
                LOG_INFO("%s", wcetLine);
            }
        }
#endif

#if ENABLE_STACK_WATERMARK
        /* Imprime watermark de pilha das tarefas */
        vPrintStackWatermarks();
#endif

        vTaskDelay(pdMS_TO_TICKS(RTOS_MONITOR_INTERVAL_MS));
    }
}