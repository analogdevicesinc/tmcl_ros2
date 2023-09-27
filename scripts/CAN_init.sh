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
    -b)
      IF_BR="$2"
      shift
      shift
      ;;
    -h)
      echo "Usage ./CAN_init.sh [<options>]"
      echo "where <options> are:"
      echo "-i [Interface Name] "
      echo "   ${SP} -CAN Interface name (e.g. can0)"
      echo "-b [Bit Rate] "
      echo "   ${SP} -CAN Bit Rate"
      echo "   ${SP}${SP}${SP}${SP}where [Bit Rate] can be the following:"
      echo "   ${SP}${SP}${SP}${SP} 20000"
      echo "   ${SP}${SP}${SP}${SP} 50000"
      echo "   ${SP}${SP}${SP}${SP} 100000"
      echo "   ${SP}${SP}${SP}${SP} 125000"
      echo "   ${SP}${SP}${SP}${SP} 250000"
      echo "   ${SP}${SP}${SP}${SP} 500000"
      echo "   ${SP}${SP}${SP}${SP} 1000000"
      echo "-h ${SP} - Get a bit of help about this script"
      echo ""
      echo "Examples:"
      echo "sudo ./CAN_init.sh -i can0 -b 1000000"
      echo "sudo ./CAN_init.sh -i can1 -b 500000"
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
elif [ -z $IF_BR ]; then
  echo "Error! Missing interface bit rate."
  echo "Try sudo $0 -h"
  exit
fi

if [ $IF_BR != 20000 -a $IF_BR != 50000 -a $IF_BR != 100000 -a \
    $IF_BR != 125000 -a $IF_BR != 250000 -a $IF_BR != 500000 -a \
    $IF_BR != 1000000 ]
then
  echo "Error! Invalid bit rate."
  echo "Try sudo $0 -h"
  exit
fi

# Create a *.network file
NETWORK_FILE="99-${IF_NAME}.network"
echo "Creating $NETWORK_FILE ..."
touch $NETWORK_FILE
echo "[Match]"        >> $NETWORK_FILE
echo "Name=$IF_NAME"  >> $NETWORK_FILE
echo "[CAN]"          >> $NETWORK_FILE
echo "BitRate=$IF_BR" >> $NETWORK_FILE

# Copy *.network file to /etc/systemd/network
echo "Moving $NETWORK_FILE to /etc/systemd/network ..."
cp $NETWORK_FILE /etc/systemd/network

# Stop systemd-networkd
echo "Stop systemd-networkd..."
systemctl stop systemd-networkd
sleep 5

# Enable systemd-networkd
echo "Enabling systemd-networkd..."
systemctl enable systemd-networkd
sleep 5

# Restart systemd-networkd
echo "Restarting systemd-networkd..."
systemctl restart systemd-networkd
sleep 5

echo "Removing temporary files..."
rm $NETWORK_FILE

echo "Done."
echo "$IF_NAME will automatically be brought UP on boot."

echo "================================================================================"
