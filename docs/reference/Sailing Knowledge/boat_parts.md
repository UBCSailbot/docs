# Parts of a Sailboat

In order to write high-quality software for autonomous sailboats, it is extremely helpful to have an
understanding of the parts of the boat and their functions.
Read the descriptions of the parts below, and refer to the image to see where the part fits in.

![image](../../../assets\images\Sailing Knowledge\parts_of_boat.jpg)

## Hull

The **Hull** is the "boat" part of the boat, which displaces water to create buoyancy. The following parts of the boat
are attached to the hull:

- **Keel**: The keel has a large mass on the end, which keeps the sailboat upright. The fin-like shape of the
keel provides "lateral resistance" to prevent the boat from slipping sideways through the water.
- **Rudders**: Raye has two rudders for redundancy. The rudders can angle side to side to steer the boat.
The rudders cannot steer the boat if the water is not flowing quickly enough over the rudders to create a pressure difference.

It is also helpful to know the names of the following "regions" of the hull:

- Bow: The front of the boat.
    - "Fore" means "towards the bow".
- Stern: The back of the boat.
    - "Aft" means "towards the stern".
- Starboard: The side of the boat which is on the _right_, for someone standing on the boat facing the bow.
- Port: The side of the boat which is on the _left_, for someone standing on the boat facing the bow.
    - To remember which is which between starboard and port, remember that "port" and "left" both have 4 letters.

![image](../../../assets\images\Sailing Knowledge\regions_of_hull.jpg)

## Jib

The **Jib** is the smaller of the two sails, which is located in front of the mast.

- **Jib Sheet**: In general, "sheets" are ropes that pull a sail in to the boat. The jib Sheet pulls in the jib.
- The **Jib Winch** is a motor-driven device that tightens or pulls in the jib sheet.
- The "jib halyard": In general, a "halyard" is a rope that pulls a sail up. The jib halyard pulls up the jib.
It connects to the top of the jib, runs through a pulley near the top of the mast, and is tied off
near the bottom of the mast.

## Mast

The **Mast** is the long pole which connects to hull. It holds up the sails and some instruments.

The following instruments are at the top of the mast:

- One of the 3 **Wind Sensors**. The top of the mast is a good location to measure undisturbed wind.
- The **AIS** antenna. AIS stands for "Autonomous Identification System" and is a system by which ships
communicate their location, speed, and other information to surrounding ships via radio signals.
Pathfinding uses AIS data to avoid other ships.

The mast is held upright by three lines:

- The "forestay" connects the mast from the top of the jib to the bow, and runs parallel to the front edge of the jib.
- The two "shrouds" connect the mast from the top of the jib to the outside edges of the hull slightly aft of the mast.
There is one shroud on the startboard side and one on the port side.

## Main Sail

The Main Sail is the larger of the two sails, and is located aft of the mast.
Most of the boat's propulsion comes from the main sail.

- The **Boom** is the name for the pole that holds the bottom corner of the main sail out from the mast.
- The "main halyard" is the line used to hoist the main sail.
- **Main Sheet** is the rope that pulls the main sail in towards the center of the boat.
- The **Main Winch** is a motor-driven device that pulls on the end of the main sheet.
