#Coffee-robot
This is the MTA Coffee-robot Repository


==========================================

Install Odrive:
-Install Python 3. (for example, on Ubuntu, sudo apt install python3 python3-pip)
-Install the ODrive tools by opening a terminal and typing sudo pip3 install odrive Enter

Install OCC Headmelted (VSC for linux)
wget https://packagecloud.io/headmelted/codebuilds/gpgkey -O - | sudo apt-key add -
curl -L https://code.headmelted.com/installers/apt.sh | sudo bash
sudo apt-get install code-oss=1.29.0-1539702286
sudo apt-mark hold code-oss

CheatSheet - Setup Github on Visual Studio Code
APRIL 03, 2018 IN INSTALL, TIP
I usually access github from within Visual Studio Code.  As such, when I start coding a new project, I often need a reminder, or a cheat sheet for how to connect Visual Studio Code to a Git repository.  These notes are more for me than for anyone else, but I'm sharing them nonetheless.

Steps:
Create a directory on the local file system.
Create a repo on Github.
Select Clone "Clone or download" on Github, copy the link
In Visual Studio Code, sect File -> Add Folder to Workspace -> Select the newly created directory
Select Terminal Window
In the window, type:
git config --global user.name <github userID>
git config --global user.email <github user emailadress>

git clone <URL from github link copied earlier>
That should be all that's required.  any newly created file should be available on github after stage/commit/push.

 
