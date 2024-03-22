# Enviroment SetUp for STM32F13C8T6 from Bare Metal
In this document we are going to see how to install all the programs and extensions needed for this project.

<small>*Note: if you forgot to select the add-to-path option in any installation in the Make part you can see how to add it manually.*

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




<br><br>

---
## MAKE 
---

1. Go to https://gnuwin32.sourceforge.net/packages/make.htm scroll down and click on **Setup program** and wait for it to download.

2. Run the installer and follow the steps **(just click on next)**.

3. You need to add this program to the enviroment path.

4. Go to the installation folder and copy the path, usually it will be **C:\Program Files (x86)\GnuWin32\bin**

5. Go to **Edit the system environment variables** (type it in the windows search engine).
   
    ![Texto alternativo](Images\EnvironmentVariables.png)

6. In **Advanced** click on **Environment Variables...**.

    ![Texto alternativo](Images\EnvironmentVariables2.png)

7. In the new window select **Path** click on **Edit...**.

    ![Texto alternativo](Images\EnvironmentVariables3.png)

8. Click on **New** and paste the copied intallation path. 

    ![Texto alternativo](Images\EnvironmentVariables4.png)


9. Then only click **Ok** in all the windows to save.

10. To test if it installed correctly go to the CMD and type this comand.

        make --version
        
    *if everything is correct, it should display something like this*
    
    ![Texto alternativo](Images\Make_test.png)