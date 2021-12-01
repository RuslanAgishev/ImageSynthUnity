Suburban Neighborhood House Pack (Modular)

Example scenes uses Standard Assets packs to give it a better look and playable possibilities.
Standard assets used are characters pack for firstpersoncharacter controller,
Effects pack for some camera effects and Cinematic Image Effects downloaded from unity asset store for more camera effects.
If you do not wish to import these effects please do not import the Standard Assets and Editor Folders to your project.



List of modular object's folders and starting prefix and their usage.

FloorOutside Folder:
	Road: Use for Roads and sidewalks.
	Floor: Use for grass areas next to road pieces. You can also use terrain if you don't wish to use these pieces.

In_FloorsRoofs Folder:
	In_: Floors and roofs for inside the houses.

In_Walls Folder:
	MidIn: For partition walls that are inside the houses.

Roofs Folder:
	Roof: To create roofs.

Porch Folder:
	Porch: To create house porches.

Walls_OnlyExterior Folder:
	Wall: No interior in these pieces. For houses you can't get in. For example windows have curtains so you can't see inside the house.

Walls_WithInterior Folder:
	In_Wall: Use if you plan to have interior for the houses. These pieces have interior and exterior combined.




Building Streets, Walls and Roofs:
These are all modular pieces and work with snap to grid tool. Ctrl + L to open settings.

Ctrl + L to open snap to grid options. Recommended to use 0.5 0.5 0.5 values on snap options.
Everything should snap together with these options.
Only when building second floor partition walls with MidIn pieces you should change the Y value to 0.2.

WallDown pieces are for ground floor.
WallUp pieces are for second floor.

Wall, roof and floor materials can be changed to different ones as they are tilable.

Trees, bushes, grasses etc:
There are static trees and trees with bone animations. If you do not want to use Unity's built in terrain and want a better performance,
recommended is to use a few bone animated trees.

Lamps Day/Night: Change Material to Lamps_On or Lamps_Off, Enable/Disable LensFlare.

Window Pieces Day/Night: Change House_Exterior_Equip material to House_Exterior_Equip_Night for self illumination.

Fences: Tilable pieces with snapping should work on most cases.

Background houses: Lower in detail and optimized. Ment for background.

ObjExporter script: If you want to combine the walls or houses into one object. Export as obj the parent of the house from File -> Export -> Wavefront Obj.
Change the obj's material import settings to from model's material and project wide. Simply place the obj to same position as it was.

When using baked lightning, recommended to turn off lightmap static from all road and grass floor pieces or alternative is to take the detail texture off from
Floor_Grass material because lightmaps screw the seconds uv's.

Remember to set Rendering Path to Deferred and Color Space to Linear for correct results.


For questions and feedback please send email to support@finwardstudios.com