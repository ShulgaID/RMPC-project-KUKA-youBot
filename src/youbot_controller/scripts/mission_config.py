#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# mission_config.py — единая точка чтения параметров задачи.
# -----------------------------------------------------------------------------
# Все ноды (room_generator, spawn_task_objects, cube_detector, pick_place_
# mission) читают важные значения ОТСЮДА. Сам источник — файл
# config/mission_params.yaml, загруженный в namespace /mission_config
# launch-файлом.
#
# Приоритет источников при чтении cfg(group, key):
#   1) приватный параметр ноды  ~<key>           (точечное переопределение)
#   2) центральный параметр     /mission_config/<group>/<key>
#   3) значение по умолчанию default
#
# Благодаря пункту 1 любой параметр всё ещё можно переопределить из launch
# через <param name="key" .../> у конкретной ноды, не трогая общий файл.
# =============================================================================

import rospy

_NS = "/mission_config"


def cfg(group, key, default=None):
    """Прочитать параметр из центрального конфига с фолбэками.

    Приоритет: приватный параметр ноды ~<key> -> /mission_config/<group>/<key>
    -> default. ПУСТАЯ строка в приватном параметре ИГНОРИРУЕТСЯ (трактуется
    как «не задано»), чтобы launch-аргументы с default='' не затирали значения
    из единого конфига.
    """
    # 1) приватное переопределение ноды (пустую строку пропускаем)
    priv = "~" + key
    if rospy.has_param(priv):
        val = rospy.get_param(priv)
        if not (isinstance(val, str) and val == ""):
            return val
    # 2) центральный конфиг
    central = "%s/%s/%s" % (_NS, group, key)
    if rospy.has_param(central):
        return rospy.get_param(central)
    # 3) дефолт
    return default


def cfg_f(group, key, default=None):
    v = cfg(group, key, default)
    return None if v is None else float(v)


def cfg_i(group, key, default=None):
    v = cfg(group, key, default)
    return None if v is None else int(v)


def cfg_s(group, key, default=None):
    v = cfg(group, key, default)
    return None if v is None else str(v)
