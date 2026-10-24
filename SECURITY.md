# Security policy

This repository must remain independent of the physical robot. Do not commit:

- firmware, upload scripts or motor serial bridges;
- `/dev/serial/by-id` values, MAC addresses or network credentials;
- tokens, private keys, `.env` files or recorded ROS bags;
- logs, camera captures or maps collected from a private site.

Use synthetic Gazebo worlds and the included test assets for development.
Report an accidental secret through a private channel and rotate it before
removing it from Git history.
