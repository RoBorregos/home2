
# Save password to use in scripts that require sudo
echo "Please enter your sudo password:"
read -s SUDOPASSWORD

# Verify the password works
if echo "$SUDOPASSWORD" | sudo -Svp "" 2>/dev/null; then
    echo "Password verified successfully."
    export SUDO_PASSWORD="$SUDOPASSWORD"
else
    echo "Invalid password. Please run the script again."
    exit 1
fi

# Clear sudo timeout to ensure the password gets checked
sudo -k

bash zed.sh

cd ./storing
# for script in ./*.sh; do
#     if [ -f "$script" ]; then
#         echo "Executing $script"
#         bash "$script"
#     fi
# done
script="manipulation-moveit.sh"
bash $script

# wait for the script to finish (4 secs)
script="hri.sh"
bash $script
script="object-detector.sh"
bash $script
script="manipulation-pick-place.sh"
bash $script

sleep 30

script="navigation.sh"
bash $script


script="integration-point-transformer.sh"
bash $script

sleep 3

script="integration.sh"
bash $script
