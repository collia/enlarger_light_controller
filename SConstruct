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


stm32cubef4_hal_path = '../stm32cubef4/Drivers/STM32F4xx_HAL_Driver/'
stm32cubef4_cmsis_path = '../stm32cubef4/Drivers/CMSIS/'

new_generation = True

if not new_generation:
    stm_platform='stm32f1xx'
    stm_family='STM32F103XB'
    #stm_device='STM32F103X6'
    stm_device='STM32F103XB'
    stm32cube_hal_path  = stm32cubef1_hal_path
    stm32cube_cmsis_path = stm32cubef1_cmsis_path
    bsp_file = 'bsp_rev_0_1.c'
else:
    stm_platform='stm32f4xx'
    stm_family='STM32F407xx'
    stm_device='STM32F407VxTx'
    stm32cube_hal_path  = stm32cubef4_hal_path
    stm32cube_cmsis_path = stm32cubef4_cmsis_path
    bsp_file = 'bsp_rev_1_0.c'

# include locations
env.Append(CPPPATH = [
    '#inc',
    '#' + stm32cube_hal_path + 'Inc',
    '#' + stm32cube_cmsis_path+'Include',
    '#' + freertos_path + '/include',
    '#' + freertos_path + freertos_portble_path
    ])

if not new_generation:
    env.Append(CPPPATH = [
        '#' + stm32cube_cmsis_path+'Device/ST/STM32F1xx/Include'
        ])
else:
    env.Append(CPPPATH = [
        '#' + stm32cube_cmsis_path+'Device/ST/STM32F4xx/Include'
        ])


env.Append(LIBPATH = [
    'lib'
    ])

# compiler flags
if not new_generation:
    env.Append(CCFLAGS = [
        '-mcpu=cortex-m3',
        '-march=armv7-m',
        '-mthumb',
        '-Os',
        '-std=gnu11',
        '-Wall',
        '-g',
        '-DDISABLE_ECHO'
    ])
else:
    env.Append(CCFLAGS = [
        '-mcpu=cortex-m4',
        '-mthumb',
        '-std=gnu11',
        '-Wall',
        '-g',
        '-DDISABLE_ECHO'
    ])

if new_generation:
    env.Append(CCFLAGS = [
        '-DBOARD_REVISION_1_0'
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


env.VariantDir('build/stm32/', stm32cube_hal_path+'Src', duplicate=0)

if not new_generation:
    env.Library('lib/libstm32',
                   [
                       'build/stm32/stm32f1xx_hal.c',
                       'build/stm32/stm32f1xx_hal_gpio.c',
                       'build/stm32/stm32f1xx_hal_rcc.c',
                       'build/stm32/stm32f1xx_hal_cortex.c',
                       'build/stm32/stm32f1xx_hal_dma.c',
                       #'build/stm32/stm32f1xx_hal_flash.c',
                       'build/stm32/stm32f1xx_hal_gpio.c',
                       #'build/stm32/stm32f1xx_hal_pwr.c',
                       #'build/stm32/stm32f1xx_hal_uart.c',
                       'build/stm32/stm32f1xx_hal_spi.c',
                       'build/stm32/stm32f1xx_hal_tim.c',
                   ])
else:
        env.Library('lib/libstm32',
                   [
                       'build/stm32/stm32f4xx_hal.c',
                       'build/stm32/stm32f4xx_hal_gpio.c',
                       'build/stm32/stm32f4xx_hal_rcc.c',
                       'build/stm32/stm32f4xx_hal_cortex.c',
                       'build/stm32/stm32f4xx_hal_dma.c',
                       #'build/stm32/stm32f4xx_hal_flash.c',
                       'build/stm32/stm32f4xx_hal_gpio.c',
                       #'build/stm32/stm32f4xx_hal_pwr.c',
                       #'build/stm32/stm32f4xx_hal_uart.c',
                       'build/stm32/stm32f4xx_hal_spi.c',
                       'build/stm32/stm32f4xx_hal_tim.c',
                       'build/stm32/stm32f4xx_hal_tim_ex.c',
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
        'build/src/max7219.c',
        'build/src/controls.c',
        'build/src/ws2812_line.c',
        'build/src/' + stm_platform + '_it.c',
        'build/src/' + stm_platform + '_hal_msp.c',
        'build/src/system_' + stm_platform + '.c',
        'build/src/' + bsp_file,
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
