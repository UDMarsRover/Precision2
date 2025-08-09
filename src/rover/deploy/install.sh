#!/bin/bash
# install the bt service

sudo cp bt_drive.service /etc/systemd/system/bt_drive.service
sudo cp rover_base.service /etc/systemd/system/rover_base.service
sudo systemctl daemon-reload
sudo systemctl enable bt_drive.service
sudo systemctl enable rover_base.service
sudo systemctl start bt_drive.service
sudo systemctl start rover_base.service