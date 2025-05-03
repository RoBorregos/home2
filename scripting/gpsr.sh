
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

# bash zed.sh

cd ./gpsr
for script in ./*.sh; do
    if [ -f "$script" ]; then
        echo "Executing $script"
        bash "$script"
    fi
done
