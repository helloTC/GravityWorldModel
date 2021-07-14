# PhysicalEngine
## Frame structure to build physical engine and different situations.
This python repository was developed to build different situations in a simulated physical environment. All code was build inherited from [Pybullet](https://pybullet.org).

*The design principle of this repository follows: concise, readable, and modular. All modules should include rich and clear annotations. Moreover, parameters of each function should decrease in the most possibility.*

## Present Functions
We have finished the basic frame on the situation block stability. Please see [stability_func.py](https://github.com/helloTC/PhysicalEngine/blob/main/utils/stability_func.py) for details.

## TODO LIST
Further functions were not determined, which would developed following my personal project.
In short time we will pay more attention on solving cognitive mechanisms of stability inference.

## Installation and setup
You can download this toolbox from the github page.
Then you need to configure environment variable in your system.
If you're using a linux system, please install it under below step:

1. Click Clone or download, select Download ZIP from the github page [https://github.com/helloTC/PhysicalEngine](https://github.com/helloTC/PhysicalEngine).
2. Unzip your downloaded toolbox to one folder (with the directory as <your_directory>).
3. Configure the environment variable using .bashrc, do as:

```bash
$ gedit ~/.bashrc
```

In .bashrc, edit it and add:

```bash
export PYTHONPATH=$PYTHONPATH:<your_directory>
```

Exit .bashrc, in your teminal, execute:
```bash
$ source ~/.bashrc
```

Now you have installed this toolbox in your python environment.

### Usage
In your python environment, if you'd like to call functions in this toolbox, you can import it as other packages:

```python
>>> from PhysicalEngine.utils import stability_func
>>> stability_func.run_IPE?
>>> stability_func.run_IPE(boxIDs, 0.3, 0.3)
```

### How to get involved
I'm thrilled to welcome new contributors. If you have good ideas on the framework or some specific functions, please contact with me:
[taicheng_huang@mail.bnu.edu.cn](taicheng_huang@mail.bnu.edu.cn)

## Acknowledge
Thanks to Yitao Xu[study0098](https://github.com/study0098) for his wonderful beginning of this repository. I tidied his code to make this version. 

