<#
.SYNOPSIS
    ERC 2026 helper for Windows PowerShell.

.DESCRIPTION
    ROS 2 does not run on Windows. It runs inside the Docker container, which
    runs inside WSL2. So `ros2 launch ...` typed into PowerShell will always
    fail with "The term 'ros2' is not recognized" - the command has to be sent
    down into the container.

    This script does that forwarding for you.

.EXAMPLE
    .\erc.ps1 sim                 # launch the Gazebo simulation (GUI)
    .\erc.ps1 sim headless        # launch it without the GUI (much faster)
    .\erc.ps1 solution 2 red      # launch our solution
    .\erc.ps1 status              # what is running, is the clock moving
    .\erc.ps1 logs sol            # follow the solution log
    .\erc.ps1 shell               # drop into a shell inside the container
    .\erc.ps1 build               # colcon build the workspace
    .\erc.ps1 stop                # stop the sim
#>

param(
    [Parameter(Position = 0)][string]$Action = 'status',
    [Parameter(Position = 1)][string]$Arg1 = '',
    [Parameter(Position = 2)][string]$Arg2 = ''
)

$ErrorActionPreference = 'Stop'

# The repo lives in the WSL filesystem, not this Windows folder: colcon needs
# real symlinks and OneDrive must not sync build artifacts.
$WslRepo = '~/erc_sim_2026'

$cmd = "cd $WslRepo && ./tools/erc.sh $Action $Arg1 $Arg2"

Write-Host "[erc] wsl -d Ubuntu-22.04 -- bash -lc `"$cmd`"" -ForegroundColor DarkGray
wsl.exe -d Ubuntu-22.04 -- bash -lc $cmd
