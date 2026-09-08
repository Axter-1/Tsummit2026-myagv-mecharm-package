#!/usr/bin/env bash
# Requisitos compartidos por los comandos de la cadena de agarre.
# Se mantiene como shell para no introducir un parser YAML en la consola.

PREPARE_GRASP_REQUIRES_DISTRIBUTED=1
PREPARE_GRASP_DEFAULT_TABLE_MM=100
PREPARE_GRASP_REMOTE_COMMAND='./scripts/tsummit_offboard.sh prepare'
PREPARE_GRASP_READY_FILE='/workspace/log/robot_routine/tsummit-grasp-ready'
PREPARE_APPROACH_READY_FILE='/workspace/log/robot_routine/tsummit-approach-ready'
