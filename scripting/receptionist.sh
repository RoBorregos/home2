bash zed.sh

cd ./receptionist
for script in ./*.sh; do
    if [ -f "$script" ]; then
        echo "Executing $script"
        bash "$script"
    fi
done
