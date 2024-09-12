# Loading PCD Files in Unity

Loading Point Cloud Data (PCD) files into Unity can be challenging and often requires a paid solution. In this project, we've utilized the Pcx repository available on [GitHub](https://github.com/keijiro/Pcx). However, it's important to note that Pcx is primarily designed to work with `.ply` files. We recommend using CloudCompare to convert `.pcd` files to `.ply`.

## Conversion Process in CloudCompare

he first challenge of importing a PCD file into Unity is correcting the dimensions. You must apply the following transformation in the opening window when you import your file in CloudCompare.

![Example of PCD file in CloudCompare](image.png)

<Strong> Note </Strong>: Sometimes, you may not find a similar opening. In this case, first save your model as a PLY file using CloudCompare, then load the PLY file again and continue with the steps below.

When loading a PCD file into CloudCompare, pay close attention to the XYZ transformations, as illustrated below:

![XYZ Transformation settings](image-1.png)

Ensure that you accurately fill out the input fields based on your data:

![Input fields for transformation settings](image-2.png)

After configuring the transformations, save the file in the `.ply` binary format.

## Integrating .PLY Files into Unity

To use the converted `.ply` files in Unity, you will first need to install PCX. Begin by locating the `manifest.json` file in the Unity project’s `Packages` folder.

![Location of manifest.json](image-3.png)

Once PCX is installed, you can drag and drop the `.ply` files into your Unity environment. However, for optimal performance, it is advisable to clean up previously added project files. This practice ensures a smoother engine performance and enhances the quality and placement of 3D models prior to conducting simulation tests.



![Unity with PCD](image-4.png)


