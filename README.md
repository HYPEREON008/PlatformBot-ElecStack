#This is the read me File 

I will give the instructions to ssh your github so you can directly upload 


do 

1. sudo apt install git 
2. git config --global user.name "Your Name"
3. git config --global user.email "your_email@example.com"
4. ssh-keygen -t ed25519 -C "your_email@example.com"
5. cat ~/.ssh/id_ed25519.pub
6. ssh -T git@github.com

This will initialise and give you your SSH key to for the github

Go to github and then add go to settings there you will find SSH ang GPG keys and under the SSH and GPG, add your SSH key

This is the basic code to create a project and add 

mkdir myproject && cd myproject
git init
echo "# My Project" > README.md
git add .
git commit -m "Initial commit"
git branch -M main
git remote add origin git@github.com:USERNAME/REPO.git
git push -u origin main



git clone git@github.com:mad-hav-22-07/REPO_NAME.git

REPO_NAME = Elec_stack_Platform_Bot
