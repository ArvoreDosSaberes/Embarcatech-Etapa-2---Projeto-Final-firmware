# Tutorial: Funções de Callback (Hooks) do FreeRTOS

Este tutorial explica como ativar e utilizar as principais funções de callback do FreeRTOS para tratamento de erros e alocação estática de memória.

---

## Índice

1. [Visão Geral](#visão-geral)
2. [Configuração no FreeRTOSConfig.h](#configuração-no-freertoscconfigh)
3. [vApplicationStackOverflowHook](#vapplicationstackoverflowhook)
4. [vApplicationMallocFailedHook](#vapplicationmallocfailedhook)
5. [vApplicationGetIdleTaskMemory](#vapplicationgetidletaskmemory)
6. [vApplicationGetTimerTaskMemory](#vapplicationgettimertaskmemory)
7. [Stack Watermark - Monitoramento de Pilha](#stack-watermark---monitoramento-de-pilha)
8. [Estratégias de Uso e Boas Práticas](#estratégias-de-uso-e-boas-práticas)
9. [Exemplo Completo](#exemplo-completo)

---

## Visão Geral

O FreeRTOS utiliza um sistema de **callbacks** (também chamados de *hooks*) para permitir que a aplicação personalize o comportamento do kernel em situações específicas. Essas funções são chamadas automaticamente pelo FreeRTOS quando determinados eventos ocorrem.

### Tipos de Callbacks

| Callback | Propósito |
|----------|-----------|
| `vApplicationStackOverflowHook` | Detecção de estouro de pilha |
| `vApplicationMallocFailedHook` | Tratamento de falha de alocação de memória |
| `vApplicationGetIdleTaskMemory` | Fornecimento de memória estática para a tarefa Idle |
| `vApplicationGetTimerTaskMemory` | Fornecimento de memória estática para a tarefa Timer |

---

## Configuração no FreeRTOSConfig.h

Para ativar cada callback, é necessário definir macros específicas no arquivo `FreeRTOSConfig.h`:

```c
/* Habilita detecção de estouro de pilha (1 ou 2) */
#define configCHECK_FOR_STACK_OVERFLOW          2

/* Habilita callback de falha de malloc */
#define configUSE_MALLOC_FAILED_HOOK            1

/* Habilita alocação estática (requer as funções Get*Memory) */
#define configSUPPORT_STATIC_ALLOCATION         1

/* Habilita software timers (requer vApplicationGetTimerTaskMemory) */
#define configUSE_TIMERS                        1
```

### Tabela de Dependências

| Macro | Valor | Callback Requerido |
|-------|-------|-------------------|
| `configCHECK_FOR_STACK_OVERFLOW` | 1 ou 2 | `vApplicationStackOverflowHook` |
| `configUSE_MALLOC_FAILED_HOOK` | 1 | `vApplicationMallocFailedHook` |
| `configSUPPORT_STATIC_ALLOCATION` | 1 | `vApplicationGetIdleTaskMemory` |
| `configUSE_TIMERS` + `configSUPPORT_STATIC_ALLOCATION` | 1 | `vApplicationGetTimerTaskMemory` |

---

## vApplicationStackOverflowHook

### O que é?

Callback chamado automaticamente pelo FreeRTOS quando detecta que uma tarefa ultrapassou os limites de sua pilha. Este é um dos erros mais comuns em sistemas embarcados e pode causar comportamento imprevisível.

### Assinatura

```c
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName);
```

### Parâmetros

| Parâmetro | Tipo | Descrição |
|-----------|------|-----------|
| `xTask` | `TaskHandle_t` | Handle da tarefa que causou o estouro |
| `pcTaskName` | `char*` | Nome da tarefa (string) |

### Ativação

No `FreeRTOSConfig.h`, defina:

```c
#define configCHECK_FOR_STACK_OVERFLOW    2
```

### Níveis de Verificação

- **Nível 1**: Verifica apenas se o ponteiro de pilha saiu dos limites. Mais rápido, mas menos confiável.
- **Nível 2**: Além da verificação do nível 1, preenche a pilha com um padrão conhecido (0xA5) e verifica se os últimos 16 bytes foram sobrescritos. Mais lento, porém mais confiável.

> **Recomendação**: Use nível 2 durante desenvolvimento e depuração.

### Implementação Recomendada

```c
/**
 * @brief Callback de estouro de pilha.
 *
 * Chamado quando uma tarefa excede o limite de sua pilha.
 * Em produção, pode-se registrar o erro e tentar reiniciar o sistema.
 *
 * @param xTask Handle da tarefa problemática.
 * @param pcTaskName Nome da tarefa.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    (void)xTask;  /* Evita warning de parâmetro não utilizado */

    /* Opção 1: Log do erro (debug) */
    printf("ERRO: Stack overflow na tarefa '%s'\r\n", pcTaskName);

    /* Opção 2: Desabilitar interrupções e parar (seguro) */
    taskDISABLE_INTERRUPTS();
    
    /* Opção 3: Acender LED de erro */
    /* HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET); */

    /* Loop infinito - sistema travado */
    for (;;)
    {
        /* Em debug, pode-se adicionar um breakpoint aqui */
    }
}
```

### Estratégias de Uso

1. **Durante desenvolvimento**: Utilize `printf` ou UART para identificar a tarefa problemática.
2. **Em produção**: Grave o nome da tarefa em memória não-volátil (Flash/EEPROM) e reinicie o sistema.
3. **Monitoramento**: Use `uxTaskGetStackHighWaterMark()` periodicamente para detectar tarefas com pilha quase esgotada.

---

## vApplicationMallocFailedHook

### O que é?

Callback invocado quando uma alocação dinâmica de memória (`pvPortMalloc`) falha por falta de heap disponível. Indica que o sistema está com memória esgotada.

### Assinatura

```c
void vApplicationMallocFailedHook(void);
```

### Ativação

No `FreeRTOSConfig.h`, defina:

```c
#define configUSE_MALLOC_FAILED_HOOK    1
```

### Implementação Recomendada

```c
/**
 * @brief Callback de falha de alocação de memória.
 *
 * Chamado quando pvPortMalloc() retorna NULL por falta de heap.
 * Indica problema grave de dimensionamento de memória.
 */
void vApplicationMallocFailedHook(void)
{
    /* Log do erro (se UART disponível) */
    printf("ERRO: Falha de alocação de memória (heap esgotado)\r\n");

    /* Informações úteis para debug */
    size_t freeHeap = xPortGetFreeHeapSize();
    size_t minEverFree = xPortGetMinimumEverFreeHeapSize();
    printf("  Heap livre: %u bytes\r\n", (unsigned int)freeHeap);
    printf("  Mínimo histórico: %u bytes\r\n", (unsigned int)minEverFree);

    /* Desabilitar interrupções e parar */
    taskDISABLE_INTERRUPTS();

    for (;;)
    {
        /* Sistema travado */
    }
}
```

### Estratégias de Uso

1. **Dimensionar corretamente o heap**: Ajuste `configTOTAL_HEAP_SIZE` baseado no uso real.
2. **Monitorar heap**: Use `xPortGetFreeHeapSize()` e `xPortGetMinimumEverFreeHeapSize()` regularmente.
3. **Preferir alocação estática**: Para sistemas críticos, use `configSUPPORT_STATIC_ALLOCATION` e elimine alocações dinâmicas.
4. **Evitar fragmentação**: Prefira `heap_4.c` ou `heap_5.c` que coalescem blocos livres.

---

## vApplicationGetIdleTaskMemory

### O que é?

Função obrigatória quando `configSUPPORT_STATIC_ALLOCATION` está habilitado. O FreeRTOS chama esta função durante `vTaskStartScheduler()` para obter a memória necessária para criar a tarefa Idle.

### Assinatura

```c
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                                   StackType_t** ppxIdleTaskStackBuffer,
                                   uint32_t* pulIdleTaskStackSize);
```

### Parâmetros

| Parâmetro | Tipo | Descrição |
|-----------|------|-----------|
| `ppxIdleTaskTCBBuffer` | `StaticTask_t**` | Ponteiro para receber o endereço do TCB (Task Control Block) |
| `ppxIdleTaskStackBuffer` | `StackType_t**` | Ponteiro para receber o endereço da pilha |
| `pulIdleTaskStackSize` | `uint32_t*` | Ponteiro para receber o tamanho da pilha (em palavras) |

### Ativação

No `FreeRTOSConfig.h`, defina:

```c
#define configSUPPORT_STATIC_ALLOCATION    1
```

### Implementação Recomendada

```c
/**
 * @brief Fornece memória estática para a tarefa Idle.
 *
 * Chamado automaticamente pelo FreeRTOS durante vTaskStartScheduler().
 * A memória deve ser estática e persistir durante toda a execução.
 *
 * @param ppxIdleTaskTCBBuffer Recebe ponteiro para o TCB.
 * @param ppxIdleTaskStackBuffer Recebe ponteiro para a pilha.
 * @param pulIdleTaskStackSize Recebe tamanho da pilha (em StackType_t).
 */
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                                   StackType_t** ppxIdleTaskStackBuffer,
                                   uint32_t* pulIdleTaskStackSize)
{
    /* Buffers estáticos - alocados em tempo de compilação */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    /* Retorna os ponteiros para os buffers */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
```

### Pontos Importantes

1. **Variáveis estáticas**: Os buffers DEVEM ser `static` para persistir após o retorno da função.
2. **Tamanho da pilha**: Use `configMINIMAL_STACK_SIZE` definido em `FreeRTOSConfig.h`.
3. **TCB (Task Control Block)**: Estrutura interna do FreeRTOS que armazena o estado da tarefa.

---

## vApplicationGetTimerTaskMemory

### O que é?

Similar à função anterior, mas fornece memória para a tarefa de serviço de timers (Timer Service Task ou *Daemon Task*). Esta tarefa processa os comandos de software timers.

### Assinatura

```c
void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
                                    StackType_t** ppxTimerTaskStackBuffer,
                                    uint32_t* pulTimerTaskStackSize);
```

### Ativação

No `FreeRTOSConfig.h`, defina:

```c
#define configSUPPORT_STATIC_ALLOCATION    1
#define configUSE_TIMERS                   1
```

### Implementação Recomendada

```c
/**
 * @brief Fornece memória estática para a tarefa Timer (daemon).
 *
 * Chamado automaticamente quando software timers são utilizados.
 * A pilha deve ser maior que a da tarefa Idle se callbacks de
 * timer executarem operações complexas.
 *
 * @param ppxTimerTaskTCBBuffer Recebe ponteiro para o TCB.
 * @param ppxTimerTaskStackBuffer Recebe ponteiro para a pilha.
 * @param pulTimerTaskStackSize Recebe tamanho da pilha (em StackType_t).
 */
void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
                                    StackType_t** ppxTimerTaskStackBuffer,
                                    uint32_t* pulTimerTaskStackSize)
{
    /* Buffers estáticos */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    /* Retorna os ponteiros */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
```

### Configurações Relacionadas

```c
/* Prioridade da tarefa Timer - geralmente alta */
#define configTIMER_TASK_PRIORITY       ( configMAX_PRIORITIES - 1 )

/* Tamanho da fila de comandos de timer */
#define configTIMER_QUEUE_LENGTH        10

/* Tamanho da pilha (deve acomodar callbacks de timer) */
#define configTIMER_TASK_STACK_DEPTH    ( configMINIMAL_STACK_SIZE * 2 )
```

---

## Stack Watermark - Monitoramento de Pilha

### O que é?

O **Stack High Water Mark** (marca d'água de pilha) é um recurso do FreeRTOS que registra o **menor espaço livre** que restou na pilha de uma tarefa desde sua criação. Funciona como um "termômetro" que indica quão próximo a tarefa chegou de um estouro de pilha.

O FreeRTOS preenche a pilha com um valor padrão (0xA5A5A5A5) durante a criação da tarefa. Quando você consulta o watermark, o kernel conta quantos bytes no final da pilha ainda mantêm esse padrão — indicando que nunca foram utilizados.

### Por que usar?

- **Dimensionamento preciso**: Ajuda a descobrir se você alocou pilha demais (desperdício) ou de menos (risco).
- **Detecção preventiva**: Identifica tarefas com pilha quase esgotada antes de ocorrer um crash.
- **Otimização de memória**: Permite reduzir pilhas superdimensionadas e liberar RAM.

### Configuração no FreeRTOSConfig.h

```c
/* Habilita a função uxTaskGetStackHighWaterMark() */
#define INCLUDE_uxTaskGetStackHighWaterMark     1

/* Habilita trace facility para uxTaskGetSystemState() */
#define configUSE_TRACE_FACILITY                1

/* Habilita registro do endereço alto da pilha (necessário para cálculo de %) */
#define configRECORD_STACK_HIGH_ADDRESS         1
```

### Funções Principais

#### uxTaskGetStackHighWaterMark()

Retorna o watermark de uma tarefa específica.

```c
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t xTask);
```

| Parâmetro | Descrição |
|-----------|-----------|
| `xTask` | Handle da tarefa (NULL = tarefa atual) |
| **Retorno** | Menor espaço livre em **words** (StackType_t) |

**Exemplo:**
```c
void vMinhaTask(void *pvParameters)
{
    for (;;)
    {
        /* ... código da tarefa ... */
        
        UBaseType_t watermark = uxTaskGetStackHighWaterMark(NULL);
        printf("Pilha livre mínima: %u words\r\n", watermark);
        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
```

#### uxTaskGetSystemState()

Obtém informações detalhadas de **todas** as tarefas do sistema.

```c
UBaseType_t uxTaskGetSystemState(
    TaskStatus_t * const pxTaskStatusArray,
    const UBaseType_t uxArraySize,
    configRUN_TIME_COUNTER_TYPE * const pulTotalRunTime
);
```

A estrutura `TaskStatus_t` contém:
- `pcTaskName` - Nome da tarefa
- `usStackHighWaterMark` - Watermark da pilha
- `pxStackBase` - Endereço base da pilha
- `pxEndOfStack` - Endereço do topo da pilha (se `configRECORD_STACK_HIGH_ADDRESS=1`)

### Implementação de Monitor de Watermark

Este projeto inclui um módulo de monitoramento completo em `tasks/rtos_monitor.c`. Para ativá-lo:

#### 1. Defina no env.cmake:

```cmake
set(ENV{ENABLE_STACK_WATERMARK} "1")
```

#### 2. Saída Gerada:

```
==================== Stack Watermark Report ====================
Tarefa           |    Total |    Livre |    Usado | Uso%  | Status
-----------------------------------------------------------------
rtos_monitor     |     3072 |     1840 |     1232 |   40% | OK
network_poll     |     1024 |      580 |      444 |   43% | OK
temp_hum_task    |     2048 |      320 |     1728 |   84% | ATENCAO
kbd_task         |     2048 |      140 |     1908 |   93% | CRITICO
-----------------------------------------------------------------
TOTAL            |     8192 |     2880 |     5312 |   64%
Tarefas: 4 | Stack total: 8192 bytes | Uso global: 64%
==================== Fim do Report ====================
```

#### 3. Interpretação dos Status:

| Status | Uso% | Ação Recomendada |
|--------|------|------------------|
| **OK** | < 75% | Pilha bem dimensionada |
| **ATENCAO** | 75-89% | Considerar aumentar pilha |
| **CRITICO** | ≥ 90% | **Aumentar pilha imediatamente** |

### Cálculo do Tamanho de Pilha

Quando `configRECORD_STACK_HIGH_ADDRESS=1`, é possível calcular o tamanho total:

```c
size_t stackTotal = pxEndOfStack - pxStackBase; /* Em words */
size_t stackUsed  = stackTotal - watermark;
uint8_t percent   = (stackUsed * 100) / stackTotal;
```

> **Nota**: O valor é em **words** (StackType_t). No RP2040, 1 word = 4 bytes.

### Boas Práticas de Dimensionamento

1. **Regra dos 20%**: Mantenha pelo menos 20% de folga na pilha.
2. **Monitoramento em desenvolvimento**: Sempre compile com watermark ativo durante testes.
3. **Callbacks de timer**: Tarefas que usam timers precisam de pilha extra para os callbacks.
4. **Funções recursivas**: Evite ou limite a profundidade máxima.
5. **Variáveis locais grandes**: Prefira alocação dinâmica ou estática global para arrays grandes.

### Macro de Compilação Condicional

O módulo `rtos_monitor.c` usa compilação condicional:

```c
#if ENABLE_STACK_WATERMARK
static void vPrintStackWatermarks(void)
{
    /* ... implementação ... */
}
#endif
```

Isso permite **desativar** o monitoramento em builds de produção para economizar recursos.

---

## Estratégias de Uso e Boas Práticas

### 1. Alocação Estática vs Dinâmica

| Aspecto | Estática | Dinâmica |
|---------|----------|----------|
| **Previsibilidade** | Alta | Baixa (fragmentação) |
| **Uso de RAM** | Fixo | Variável |
| **Falhas em runtime** | Impossível | Possível |
| **Flexibilidade** | Baixa | Alta |

> **Recomendação**: Para sistemas críticos (automotivo, médico, industrial), prefira alocação estática.

### 2. Dimensionamento de Pilha

Utilize o módulo `rtos_monitor` para análise detalhada:

```c
/* Parâmetros em rack_inteligente_parametros.h */
#define RTOS_MONITOR_STACK_SIZE         (configMINIMAL_STACK_SIZE * 3)
#define RTOS_MONITOR_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)
#define RTOS_MONITOR_MAX_TASKS          (20)
#define RTOS_MONITOR_INTERVAL_MS        (30000)
```

**Criação da tarefa:**
```c
xTaskCreate(vRTOSMonitorTask, "rtos_monitor",
            RTOS_MONITOR_STACK_SIZE,
            NULL, RTOS_MONITOR_TASK_PRIORITY,
            NULL);
```

**Verificação individual:**
```c
void vMinhaTask(void* pvParameters)
{
    for (;;)
    {
        /* ... código da tarefa ... */
        
        UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
        LOG_DEBUG("[MinhaTask] Watermark: %u words", hwm);
        
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
```

### 3. Tratamento de Erros em Produção

```c
/* Estrutura para log de erros em Flash */
typedef struct {
    uint32_t timestamp;
    uint8_t  errorType;      /* 1=StackOverflow, 2=MallocFailed */
    char     taskName[16];
    uint32_t freeHeap;
} ErrorLog_t;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    ErrorLog_t log = {0};
    
    log.timestamp = xTaskGetTickCount();
    log.errorType = 1;
    strncpy(log.taskName, pcTaskName, sizeof(log.taskName) - 1);
    
    /* Salvar em Flash antes de reiniciar */
    Flash_WriteErrorLog(&log);
    
    /* Reiniciar sistema */
    NVIC_SystemReset();
}
```

### 4. Checklist de Configuração

- [ ] `configCHECK_FOR_STACK_OVERFLOW` = 2 (desenvolvimento)
- [ ] `configUSE_MALLOC_FAILED_HOOK` = 1
- [ ] `configSUPPORT_STATIC_ALLOCATION` = 1 (sistemas críticos)
- [ ] `configUSE_TRACE_FACILITY` = 1 (para watermark detalhado)
- [ ] `configRECORD_STACK_HIGH_ADDRESS` = 1 (para cálculo de %)
- [ ] `INCLUDE_uxTaskGetStackHighWaterMark` = 1
- [ ] Implementar todos os callbacks obrigatórios
- [ ] Ativar `ENABLE_STACK_WATERMARK=1` no env.cmake (desenvolvimento)
- [ ] Monitorar `uxTaskGetStackHighWaterMark()` em desenvolvimento
- [ ] Monitorar `xPortGetMinimumEverFreeHeapSize()` em desenvolvimento

---

## Exemplo Completo

Arquivo `freertos_hooks.c`:

```c
/**
 * @file freertos_hooks.c
 * @brief Implementação dos callbacks obrigatórios do FreeRTOS.
 */

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

/*-----------------------------------------------------------
 * Hook de estouro de pilha
 *----------------------------------------------------------*/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    (void)xTask;
    
    printf("FATAL: Stack overflow em '%s'\r\n", pcTaskName);
    
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

/*-----------------------------------------------------------
 * Hook de falha de malloc
 *----------------------------------------------------------*/
void vApplicationMallocFailedHook(void)
{
    printf("FATAL: Malloc failed. Heap livre: %u\r\n", 
           (unsigned int)xPortGetFreeHeapSize());
    
    taskDISABLE_INTERRUPTS();
    for (;;) { }
}

/*-----------------------------------------------------------
 * Memória estática para tarefa Idle
 *----------------------------------------------------------*/
void vApplicationGetIdleTaskMemory(StaticTask_t** ppxIdleTaskTCBBuffer,
                                   StackType_t** ppxIdleTaskStackBuffer,
                                   uint32_t* pulIdleTaskStackSize)
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/*-----------------------------------------------------------
 * Memória estática para tarefa Timer
 *----------------------------------------------------------*/
void vApplicationGetTimerTaskMemory(StaticTask_t** ppxTimerTaskTCBBuffer,
                                    StackType_t** ppxTimerTaskStackBuffer,
                                    uint32_t* pulTimerTaskStackSize)
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
```

---

## Referências

- [FreeRTOS - Stack Overflow Detection](https://www.freertos.org/Stacks-and-stack-overflow-checking.html)
- [FreeRTOS - Stack Usage and Stack Overflow Checking](https://www.freertos.org/Stacks-and-stack-overflow-checking.html)
- [FreeRTOS - uxTaskGetStackHighWaterMark()](https://www.freertos.org/uxTaskGetStackHighWaterMark.html)
- [FreeRTOS - uxTaskGetSystemState()](https://www.freertos.org/uxTaskGetSystemState.html)
- [FreeRTOS - Static vs Dynamic Memory Allocation](https://www.freertos.org/Static_Vs_Dynamic_Memory_Allocation.html)
- [FreeRTOS - Memory Management](https://www.freertos.org/a00111.html)
- [FreeRTOS - Software Timers](https://www.freertos.org/FreeRTOS-Software-Timer-API-Functions.html)

---

## Arquivos Relacionados no Projeto

| Arquivo | Descrição |
|---------|----------|
| `FreeRTOSConfig.h` | Configurações do kernel FreeRTOS |
| `tasks/rtos_monitor.c` | Implementação do monitor de watermark |
| `tasks/rtos_monitor.h` | Header do monitor com documentação |
| `inc/rack_inteligente_parametros.h` | Parâmetros de pilha das tarefas |
| `CMakeLists.txt` | Macro `ENABLE_STACK_WATERMARK` |

---

*Documento atualizado para o projeto Rack Inteligente - EmbarcaTech 2025*
