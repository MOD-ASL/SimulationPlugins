if [ $1 = '-w' ]; then
    cd ~/.gazebo/models/SMORES8Jack/
    if [ -n "$2" ]; then
        gazebo $2
    else
        gazebo World_sim.sdf
    fi
fi

