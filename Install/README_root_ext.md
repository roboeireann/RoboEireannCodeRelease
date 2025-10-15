# RoboEireann root image required to flash robots

To create a flash image for the robots you need to run the `deploy -i` script.
This script depends on the RoboEireann root image, `root.ext3` or its compressed
version `root.ext3.gz`.

You can create a new root.ext3 via the createRootImage script in the rematch 
Install directory. This is straightforward but a bit time consuming. Refer to 
the [B-Human instructions](https://docs.b-human.de/coderelease2021/getting-started/#creating-a-root-image)
for more information if you want to go ahead and do this.

If you want to skip creating the root image yourself, then download 
root.ext3.gz from the RoboEireann shared onedrive/sharepoint folder, 
specifically from the `roboeireann_root_image/2024-12-07` subdirectory.

Copy the downloaded file into the `Install` directory of your rematch project.

That should be it and you should not be able to run the deploy script to 
generate a new flash image for the robot based on our current software, e.g.

    Make/Linux/deploy Develop -i -v 50 -w ASUS_5_RE_LAB



