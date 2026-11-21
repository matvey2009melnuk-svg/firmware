#ifndef __AIR_MOUSE_H
#define __AIR_MOUSE_H
#if defined(HAS_AIR_MOUSE)

// BLE Air Mouse для M5StickC Plus2: гироскоп MPU6886 -> курсор.
// A = ЛКМ (удержание = drag), B = ПКМ, B удержать = прокрутка,
// PWR: клик = пауза, двойной клик = пресет чувствительности, удержание 1 с = выход.
void airMouseMode();

#endif
#endif
