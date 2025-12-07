#ifndef __RTOS_MONITOR_H_
#define __RTOS_MONITOR_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file rtos_monitor.h
 * @brief Monitor de sistema RTOS para depuração.
 *
 * Este módulo fornece uma tarefa de monitoramento que imprime
 * informações periódicas sobre o estado do sistema:
 * - Uso de heap (livre e mínimo histórico)
 * - Stack watermark das tarefas (quando ENABLE_STACK_WATERMARK=1)
 *
 * @section config Configuração
 * Para habilitar o relatório de watermark de pilha, defina no env.cmake:
 * @code
 * set(ENV{ENABLE_STACK_WATERMARK} "1")
 * @endcode
 *
 * @section output Saída de Watermark
 * Quando habilitado, o report de watermark mostra para cada tarefa:
 * - **Total**: Tamanho total da pilha em bytes
 * - **Livre**: Watermark (menor espaço livre desde a criação) em bytes
 * - **Usado**: Consumo máximo de pilha (total - livre) em bytes
 * - **Uso%**: Porcentagem de uso individual
 * - **Status**: OK (<75%), ATENCAO (75-89%), CRITICO (>=90%)
 *
 * Ao final, exibe totalizadores globais:
 * - Total de stack alocado para todas as tarefas
 * - Total de stack usado (pico)
 * - Porcentagem de uso global
 *
 * @note Requer configRECORD_STACK_HIGH_ADDRESS=1 no FreeRTOSConfig.h
 *       para cálculo do tamanho total da pilha.
 */

/**
 * @brief Tarefa de monitoramento do sistema RTOS.
 *
 * Monitora o estado do sistema, incluindo uso de memória e
 * estado das tarefas. Útil para debug e diagnóstico.
 *
 * Quando ENABLE_STACK_WATERMARK está ativo (definido como 1 no env.cmake),
 * também imprime o watermark de pilha de todas as tarefas ativas.
 *
 * @param pvParameters Parâmetros da tarefa (não utilizado).
 *
 * @note Esta tarefa executa a cada 30 segundos por padrão.
 * @note O watermark indica o mínimo de espaço livre na pilha.
 *       Quanto menor o valor, mais próximo do estouro de pilha.
 */
void vRTOSMonitorTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // __RTOS_MONITOR_H_