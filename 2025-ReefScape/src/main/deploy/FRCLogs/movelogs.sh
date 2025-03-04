#!/bin/bash

DEVNAM="sda1"
FRCPATH="/media/sda1/FRCLogs/"

if [ -e /dev/$DEVNAM ] && [ -d /media/$DEVNAM/ ]; then
  mkdir -p $FRCPATH

  cp /home/lvuser/*.log* $FRCPATH
  mv /home/lvuser/logs/* $FRCPATH 2>/dev/null

  umount /media/$DEVNAM
fi
