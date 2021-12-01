## City Traffic simulation
Animated pedestrians (man, woman and a child) added to the city environment. However, it is possible to choose different mecanim-ready human prefabs instead
or in addition to the existing ones, example: [Main-in-Suit](https://assetstore.unity.com/packages/3d/characters/humanoids/man-in-a-suit-51662).
Animations are added to human models with the help of the [PlayerMovementUsingAnimator](https://github.com/OBalfaqih/PlayerMovementUsingAnimator)
repository.

Number of pedestirans in the scene could be adjusted with the parameter. Humans move along sidewalks and pedestrian crossings.
The navigation logic for pedestrians is defined with the help of [NavMeshComponents](https://github.com/Unity-Technologies/NavMeshComponents)
Unity package.

City traffic also includes cars (of different color) moving along predefined trajectories on the roads. The number of cars is
defined as a parameter as well. Navigation logic is implemented with the help of [iTS](https://assetstore.unity.com/packages/templates/systems/its-intelligent-traffic-system-23564)
Unity asset.

<img src="../figures/pedestrians.gif" width="500"/> <img src="../figures/city_traffic.gif" width="500"/>


## Data-collection with a fleet of dynamic objects
In order to collect even more data with a group of dynamic agents, simply
add a camera to each of them with the following scripts attached to the camera:
[ImageSynthesis.cs](https://gitlab.com/RuslanAgishev/ImageSynthUnity/-/blob/master/Assets/ImageSynthesis/ImageSynthesis.cs)
and [DataRecorder.cs](https://gitlab.com/RuslanAgishev/ImageSynthUnity/-/blob/master/Assets/Scripts/DataRecorder.cs)
scripts.
As a result, for each game object there will be generated separate odometry and detection `json`-files
as well as images with a specific tag.

For more details, ave a look at the parser of a dataset collected from 2 individual cars,
[parse_multicam_data.ipynb](https://gitlab.com/RuslanAgishev/ImageSynthUnity/-/blob/master/tools/parse_multicam_data.ipynb).

