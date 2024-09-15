Place files for startup services in this directory.

- Service files should be placed into the services folder. 
- Scripts run by services should be placed in the scrips folder. 

When referencing a script in your service, you should use the path `/etc/udmrt/startup_scripts/<your_script.sh>`. The files from the scripts folder will be copied to this location, and the services from the service directory will be created and enabled by the `rover_setup.sh` script. 

Services that start with `udmrt` in their name will automatically be enabled.