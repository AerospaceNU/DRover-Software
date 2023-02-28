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
