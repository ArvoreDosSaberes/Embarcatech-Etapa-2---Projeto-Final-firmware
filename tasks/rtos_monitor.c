#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "rtos_monitor.h"
#include "log_vt100.h"
#include "rack_inteligente_parametros.h"


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

    for (;;)
    {
        /* Monitorar heap livre */
        size_t freeHeap = xPortGetFreeHeapSize();
        size_t minFreeHeap = xPortGetMinimumEverFreeHeapSize();

        /* Log periódico de status */
        LOG_INFO("[Monitor] Heap livre: %u bytes, Minimo: %u bytes",
                 (unsigned int)freeHeap, (unsigned int)minFreeHeap);

#if ENABLE_STACK_WATERMARK
        /* Imprime watermark de pilha das tarefas */
        vPrintStackWatermarks();
#endif

        vTaskDelay(pdMS_TO_TICKS(RTOS_MONITOR_INTERVAL_MS));
    }
}