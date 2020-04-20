# Model Insertion from Fuel

Ignition Fuel hosts hundreds of models that can easily be added to a world running in the Ignition GUI.
Adding models to a world typically means adding them into your world `sdf` file, but with Fuel you can drag and drop existing models directly into the scene.

## Prerequisites

Start by getting Ignition up and running with an empty world:

```bash
ign gazebo empty.sdf
```

The previous tutorial, [Manipulating Models](Manipulating_models), shows you how to interact with models, which might be helpful once you add some Fuel models to your world.

## Choose a Model

Fuel's model collection is at [app.ignitionrobotics.org/fuel/models](https://app.ignitionrobotics.org/fuel/models).
Head over and browse the available content.

![Fuel models preview](/blueprint/img/fuel.png)

Click on any one of the thumbnails, or use the search bar.
For example, let's find the [PR2_SDF17](https://app.ignitionrobotics.org/openrobotics/fuel/models/PR2_SDF17).

![Vent](img/pr2_detail.png)

Note that some of the model files are quite large and may take some time to download into your world.
Check that the file size on the right is reasonable for your application before moving on.

## Drag and Drop

The only thing left to do is drag and drop.
Click and hold the `Drag me!` button and do just that: drag your mouse into the GUI scene area.

![Drag and drop](img/pr2.gif)

Now you can add more models, or even load in any one of the plugins you learned about to interact with the model the same way you could with the shapes in the previous tutorial.

If you prefer to add a Fuel model permanently to an `sdf` file, you can just as easily download the code by clicking the download-arrow icon to the right of the model's image.

## Next Up

Stay tuned for new tutorials on the Ignition GUI and more!
