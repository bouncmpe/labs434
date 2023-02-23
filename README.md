# A Robotic Simulation Environment

## Get started

1. Install necessary system packages on our Ubuntu machine using the command:

   ```
   sudo apt install git python3-pip python3-venv python-is-python3
   ```

   1. `python3-pip` is the package manager for Python 3, which allows you to install and manage Python packages.
   2. `python3-venv` is the module for creating isolated Python environments, which are useful for keeping the dependencies for different projects separate.
   3. `python-is-python3` is a utility package to ensure that we use Python 3 when we type `python`.

   Note that the `sudo` command is used to run the command as a superuser, which is necessary when installing packages globally on the system. The `apt` command stands for Advanced Packaging Tool and is used to manage packages on Debian-based systems like Ubuntu.

2. Setup an isolated Python environment for CMPE434 labs using `venv` (virtual environment) module.

   ```
   python -m venv ~/venv/labs434
   ```

   1. `python -m venv` is the command to run the `venv`, which is used to create isolated Python environments using our current `python` interpreter. You can check the version of your interprete by typing `python --version`.
   2. The `~/venv/labs434` argument specifies the location and name of the virtual environment. In this case, it creates a virtual environment called `labs434` in the `.venv` directory located in the home directory of the user (indicated by the tilde `~` symbol).
   3. Check the content inside the newly created directory `labs434` created under `~/venv` by typing `ls ~/venv/labs434`. This is a standalone Python environment to store the installed packages and scripts. And you can create multiple environments as needed.

   We do this step because the robotics software development greatly suffers from the problem of managing dependencies to other software components (in this case other Python packages). Often we work on multiple projects with clashing dependency requ'rements hence isolated environments are very helpful in these situations.

3. Activate the `labs434` Python environment using the command for the current shell:

   ```
   source ~/venv/labs434/bin/activate
   ```

   When you activate a virtual environment, you are essentially changing your shell's environment so that it uses the packages and dependencies contained within that virtual environment, instead of the system-wide packages. This allows you to work with different versions of packages, without affecting other projects that may be using different versions.

   The `source` command is used to run the activate script within the current shell, allowing it to modify the shell's environment. The modified environment lasts as long as the shell session is active or until you run the `deactivate` command to exit the virtual environment.

   You need to repeat the command every time you open the terminal or add it into your `.bashrc` script.

4. Install Python packages we will use once you have activated the virtual environment

   ```
   python -m pip install mujoco
   ```

   Feel free to install other packages when you need them.

6. Now explore some high-quality MuJoCo models, which can be found DeepMind's `mujoco_menagerie` repository. For that, clone the repo into a directory named `~/cmpe434/mujoco_menagerie` as follows:

   ```
   git clone https://github.com/deepmind/mujoco_menagerie.git ~/cmpe434/mujoco_menagerie
   ```
   
7. Start the viewer for the Robotiq 2F-85 model:

   ```
   python -m mujoco.viewer --mjcf ~/cmpe434/mujoco_menagerie/robotiq_2f85/scene.xml
   ```
   
   This is the MuJoCo model of a robotic gripper together with a hanging box. Find the control tab and control the robot's position from the lever.
   
8. Go and try out other models in the repository.

9. Check out the MuJoCo documentation (https://mujoco.readthedocs.io).
