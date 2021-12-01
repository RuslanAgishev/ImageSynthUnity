### Requirements ###
* Tested with Unity 2019.4.18f1 (LTS)
* Should work on any OS officially supported by Unity

### 1．Installation of Unity 2019.4.18f1 (LTS)
First, install UnityHub with following links.
- Windows, Mac : https://unity3d.com/jp/get-unity/download
- Linux : Goto https://unity3d.com/get-unity/download and click "Download Unity Hub" button to get latest `UnityHub.AppImage`.  
  Then add execution permission for `UnityHub.AppImage` by following command.
  ```bash
  sudo chmod +x UnityHub.AppImage
  ```
  Then, run the UnityHub.AppImage
   ```bash
   ./UnityHub.AppImage
   ```
   Please certificate the LICENSE for Unity on UnityHub application (you can use them free !)

After that, choose and install Unity Editor (version : `2019.4.18f`) from archive.  
https://unity3d.com/get-unity/download/archive

**For Windows**  
You have `.exe` file from above link. Just run them.

**For Linux**  
1. Right click on `Unity Hub` button on your desired Unity Editor version, and click "Copy Lilnk Location".
2. Run `UnityHub.AppImage` by setting copied link location as the argument. Here is the example for `2019.4.18` version.
   ```bash
   ./UnityHub.AppImage unityhub://2019.4.18f1/5968d7f82152
   ```
   If you need any other version, the procedure is same.
   After above commands, the UnityHub will start to install desired version's Unity Editor!

### 4. Open ImageSynthUnity project
Finally, please open `ImageSynthUnity` package from UnityHub. (It takes more than 5 minuites at the first time, in the case)

### 5. Select the Scene file
There are Scene files in `Asset/Scenes/` directory.  
Please open the Scene file you want.

### Trouble Shooting
#### Trouble with [git lfs](https://git-lfs.github.com/)
Some problems are come from `git lfs`
- If you have not installed `git lfs`, please install them. Then, `git clone` this repository again.
- If you already have installed `git lfs`, but have a problem. Try following procedure please.
```bash
git clean -fdx
git lfs pull
```
