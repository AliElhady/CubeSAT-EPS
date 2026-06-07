# Шаг 6 — Telemetry: UART через DMA (финальная сборка)

## Файлы
- `main.c`            — точка входа
- `eps_soc.c/h`       — расчёт SOC
- `eps_fsm.c/h`       — машина состояний
- `eps_gpio.c/h`      — управление нагрузками
- `eps_flash.c/h`     — Flash persistence
- `eps_telemetry.c/h` — телеметрия UART

## Что добавилось
Передача по DMA — MCU не блокируется во время отправки.
Два режима: ASCII (отладка) и бинарный 16-байт пакет (OBC).
Адаптивный интервал: 5 с в Normal, 10 с в LowPower, 30 с в Critical.
Telem_SendNow() в каждом Enter_X() — мгновенное уведомление при смене состояния.

## Что настроить в CubeMX
- USART1 + DMA (Memory to Peripheral) → терминал/наземная станция
- USART2 + DMA → OBC
- TIM6 период = 1000 мс → Telem_TickSecond()
- ВАЖНО: MX_DMA_Init() вызывать ДО MX_USART_Init()

## Режим вывода
Переключается одной строкой в eps_telemetry.c:
  #define TELEM_MODE_ASCII    ← отладка
  #define TELEM_MODE_BINARY   ← реальный полёт

## Критерий готовности
- Пакеты приходят каждые 5 с в Normal, 30 с в Critical
- При смене состояния — пакет немедленно
- Счётчик packet_id без пропусков на 100 пакетах
- DMA: FSM продолжает работать во время передачи

## Формат ASCII пакета
```
$EPS,042,NORMAL,78%,7.85V,8.12V,COLD,ch1=1,ch2=1,chg=1,up=3600
```
