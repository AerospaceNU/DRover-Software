# DRover-Software
Codebase for NUAV's DRover project in conjunction with NURover

## Setup
To set up the repo and install needed dependancies, we use setup.py as follows
```
pip install -e .
```

This allows us to reference the `drover` package from wherever we want! It also
installs `drover_main.py` as a new terminal command which is neat.

### Quick Module Explainer
The `drover` folder is set up in the format of a python module and includes 
`__init__.py` files etc ([python module reference](https://docs.python.org/3/tutorial/modules.html)).
When a packages is imported in a relative manner, you will see a period in 
front of its name (ex `import .camera`). 

## Creating a service on the RPi
We want to run `drover_startup.sh` on boot, so we shall create a unix service.
1. Create file `/etc/systemd/system//drover.service` with the following:
```
[Unit]
Description=Drover service.

[Service]
Type=simple
User=pi
Group=pi
ExecStart=/bin/bash /home/pi/.local/bin/drover_startup.sh

[Install]
WantedBy=multi-user.target


```
1. Reload stuff with `systemctl daemon-reload` 
2. Enable it: `systemctl enable drover.service`
Also of usefulness:
- `systemctl restart drover.service`
- `systemctl status drover.service`
- [Setup for neopixels](https://github.com/jgarff/rpi_ws281x/issues/326)