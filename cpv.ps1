# Copy TeamCode package contents to ../temp
# Includes: hardware, opmodes, subsystems, robot
# Excludes: readme.md

$sourceBase = "TeamCode\src\main\java\org\firstinspires\ftc\teamcode"
$destBase = "..\virtual_robot\TeamCode\src\org\firstinspires\ftc\teamcode"

# Create destination directory if it doesn't exist
if (-not (Test-Path $destBase)) {
    New-Item -ItemType Directory -Path $destBase -Force | Out-Null
}

# Copy each required directory
$directories = @("hardware", "opmodes", "subsystems", "robot")

foreach ($dir in $directories) {
    $source = Join-Path $sourceBase $dir
    $dest = Join-Path $destBase $dir
    
    if (Test-Path $source) {
        Write-Host "Copying $dir..."
        # Remove destination if it exists to avoid nested copies
        if (Test-Path $dest) {
            Remove-Item -Path $dest -Recurse -Force
        }
        Copy-Item -Path $source -Destination $dest -Recurse -Force
    } else {
        Write-Host "Warning: $source not found"
    }
}

Write-Host "Done! Contents copied to $destBase"
