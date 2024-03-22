# Enviroment SetUp for STM32F13C8T6 from Bare Metal
In this document we are going to see how to install all the programs and extensions needed for this project.

<br><br>

---
## Visual Studio Code
---

1. Go to https://code.visualstudio.com/Download and download the version for your operating system, in this case Windows 10.

![Texto alternativo](Images\VSCode_Download.png)

2. Run the installer and follow the steps **(just click on next)**.

    *Make shure you have slected the option "Add to path"*

    ![Texto alternativo](Images\VSCode_AddToPath.jpg)




<br><br>

---
## Git
---

1. Go to https://git-scm.com/download/win and download the version for your.operating system, in this case Windows 10.

![Texto alternativo](Images\Git_Download.png)

2. Run the installer and follow the steps **(just click on next)**.


3. To configure your user, open the CMD and type these commands.

        git config --global user.name "Your name"

        git config --global user.email "example@email.com"




<br><br>

---
## GNU Arm Toolchain
---

1. Go to https://developer.arm.com/downloads/-/gnu-rm and download the version for your operating system, in this case Windows 10 (Scroll down).

![Texto alternativo](Images\GNU_Download.png)

2. Run the installer and follow the steps **(just click on next)**.

    *Make shure you have slected the option "Add path to the environment variable"*
    
    ![Texto alternativo](Images\GNU_AddToPath.jpg)

3. To test if it installed correctly go to the CMD and type this comand.

        arm-none-eabi-gcc --version
    
    *if everything is correct, it should display something like this*
    
    ![Texto alternativo](Images\GNU_test.png)
