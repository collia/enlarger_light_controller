#!python

import os
env = Environment(ENV = os.environ)
 
env['AR'] = 'arm-none-eabi-ar'
env['AS'] = 'arm-none-eabi-as'
env['CC'] = 'arm-none-eabi-gcc'
env['CXX'] = 'arm-none-eabi-g++'
env['LINK'] = 'arm-none-eabi-gcc'
env['RANLIB'] = 'arm-none-eabi-ranlib'
env['OBJCOPY'] = 'arm-none-eabi-objcopy'
env['PROGSUFFIX'] = '.elf'

stm32cubef1_hal_path = '../stm32cubef1/Drivers/STM32F1xx_HAL_Driver/'
stm32cubef1_cmsis_path = '../stm32cubef1/Drivers/CMSIS/'
freertos_path = '../FreeRTOS/Source/'
freertos_portble_path = '/portable/GCC/ARM_CM3/'

stm_family='STM32F103XB'
stm_device='STM32F103X6'

# include locations
env.Append(CPPPATH = [
    '#inc', 
    '#' + stm32cubef1_hal_path + 'Inc',
    '#' + stm32cubef1_cmsis_path+'Include',
    '#' + stm32cubef1_cmsis_path+'Device/ST/STM32F1xx/Include',
    '#' + freertos_path + '/include',
    '#' + freertos_path + freertos_portble_path
    ])

env.Append(LIBPATH = [
    'lib'
    ])

# compiler flags
env.Append(CCFLAGS = [
    '-mcpu=cortex-m3',
    '-march=armv7-m',
    '-mthumb',
    '-O0',
    '-std=gnu11',
    '-Wall',
    '-g'
    ])

# linker flags
#    '-Wl,--gc-sections,-Map=main.elf.map,-cref,-u,Reset_Handler',
env.Append(LINKFLAGS = [
    #'-mcpu=cortex-m3',
    '-mthumb',
    '-Wl,--gc-sections,-Map=main.elf.map,-cref,-u,Reset_Handler,--trace',
     '-T', 'src/gcc/linker/'+ stm_device + '_FLASH.ld'
    ]) 

# defines
env.Append(CPPDEFINES = [
    stm_family.replace('X', 'x'),
])


env.VariantDir('build/stm32/', stm32cubef1_hal_path+'Src', duplicate=0)
env.Library('lib/libstm32',
                   [
                       'build/stm32/stm32f1xx_hal.c',
                       'build/stm32/stm32f1xx_hal_gpio.c',
                       'build/stm32/stm32f1xx_hal_rcc.c',
                       'build/stm32/stm32f1xx_hal_cortex.c',
                       'build/stm32/stm32f1xx_hal_dma.c',
                       'build/stm32/stm32f1xx_hal_flash.c',
                       'build/stm32/stm32f1xx_hal_gpio.c',
                       'build/stm32/stm32f1xx_hal_pwr.c',
                       'build/stm32/stm32f1xx_hal_uart.c',
                   ])

env.VariantDir('build/freertos/', freertos_path, duplicate=0)
env.Library('lib/libfreertos',
                   [
                       'build/freertos/croutine.c',
                       'build/freertos/event_groups.c',
                       'build/freertos/list.c',
                       'build/freertos/queue.c',
                       'build/freertos/stream_buffer.c',
                       'build/freertos/tasks.c',
                       'build/freertos/timers.c',
                       'build/freertos/' + freertos_portble_path + 'port.c',
                       'build/freertos/' + 'portable/MemMang/heap_1.c',
                   ])

env.VariantDir('build/src/', 'src', duplicate=0)


#print(env.Dump())
# build everything
prg = env.Program(
    target = 'main',
    LIBS=['libstm32', 'libfreertos'],
    source = [
        'build/src/main.c',
        'build/src/stm32f1xx_it.c',
        'build/src/system_stm32f1xx.c',
        'build/src/gcc/startup_' + stm_device.lower() + '.s'
    ]
)
 
# binary file builder
def arm_generator(source, target, env, for_signature):
    return '$OBJCOPY -O binary %s %s'%(source[0], target[0])

env.Append(BUILDERS = {
    'Objcopy': Builder(
        generator=arm_generator,
        suffix='.bin',
        src_suffix='.elf'
    )
})
 
env.Objcopy(prg)

