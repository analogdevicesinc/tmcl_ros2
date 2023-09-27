#!/bin/bash

# Variable for 2 spaces
SP=`echo "  "`

if [ "$(whoami)" != root ]
then
  echo "Please run this script with sudo. Exiting!"
  echo "Try sudo $0 -h"
  exit
fi

# Script arguments check
echo "================================================================================"

while [ ! $# -eq 0 ]
do
  case "$1" in
    -i)
      IF_NAME="$2"
      shift
      shift
      ;;
    -h)
      echo "Usage ./CAN_deinit.sh [<options>]"
      echo "where <options> are:"
      echo "-i [Interface Name] "
      echo "   ${SP} -CAN Interface name (e.g. can0)"
      echo ""
      echo "Examples:"
      echo "sudo ./CAN_deinit.sh -i can0"
      echo "sudo ./CAN_deinit.sh -i can1"
      exit
      ;;
    *)
      shift
      ;;
  esac
done

# Validate inputs
if [ -z $IF_NAME ]; then
  echo "Error! Missing interface name."
  echo "Try sudo $0 -h"
  exit
fi

NETWORK_FILE="99-${IF_NAME}.network"
NETWORK_FILE_DIR="/etc/systemd/network/${NETWORK_FILE}"

if [ -f $NETWORK_FILE_DIR ]; then
  echo "Deinitializing ${NETWORK_FILE_DIR} ..."
  echo "Stop systemd-networkd ..."
  systemctl stop systemd-networkd
  sleep 5

  echo "Removing CAN network file ..."
  rm ${NETWORK_FILE_DIR}

  echo "Restarting systemd-networkd ..."
  systemctl restart systemd-networkd
  sleep 5

  echo ""
  echo "Done."
  echo "Automatic bring up of $IF_NAME is now disabled"
else
  echo "${NETWORK_FILE_DIR} does not exist."
  echo "Cannot deinitialize."
fi
echo "================================================================================"