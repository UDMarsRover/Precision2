#!/bin/bash
# install the bt service
su

cp bt_drive.service /etc/systemd/system/bt_drive.service
systemctl daemon-reload
systemctl enable bt_drive.service
systemctl start bt_drive.service