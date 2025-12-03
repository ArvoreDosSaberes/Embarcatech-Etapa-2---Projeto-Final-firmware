# Configuração central de FreeRTOS para o projeto Rack Inteligente

include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Define que o FreeRTOS está habilitado no projeto
set(FREERTOS_ENABLED ON CACHE BOOL "Enable FreeRTOS in the project")

# Escolha global de heap do FreeRTOS (1..5 ou caminho para arquivo customizado)
set(FREERTOS_HEAP "4" CACHE STRING "FreeRTOS heap model number. 1 .. 5. Or absolute path to custom heap source file")
set_property(CACHE FREERTOS_HEAP PROPERTY STRINGS "1;2;3;4;5")

# Define variáveis globais com os diretórios de include do FreeRTOS
set(FREERTOS_INCLUDE_DIRS
	${FREERTOS_KERNEL_PATH}/include
	${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include
	${CMAKE_SOURCE_DIR}
	CACHE INTERNAL "FreeRTOS include directories"
)

# Função helper centralizada para configurar FreeRTOS em um alvo
# Adiciona includes, links e dependências necessárias
function(link_freertos target)
	# Adiciona os diretórios de include do FreeRTOS
	target_include_directories(${target} PUBLIC
		${FREERTOS_KERNEL_PATH}/include
		${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include
		${CMAKE_SOURCE_DIR}
	)
	
	# Linka com o kernel do FreeRTOS
	if(TARGET freertos_kernel)
		target_link_libraries(${target} PUBLIC freertos_kernel)
	endif()
endfunction()
