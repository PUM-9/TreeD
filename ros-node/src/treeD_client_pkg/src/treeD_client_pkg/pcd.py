def create_point_cloud(obj, path, width, height, num_slices):
    """
    Function used to save the pointcloud to a PCD-file.

    :param obj: the data of the pointcloud.
    :param path: the path to where to save the file.
    :param width: the width of the pointcloud.
    :param height: the height of the pointcloud.
    :param num_slices: the number of slices the received data is divided in.
    """
    f = open(path, 'w')

    outputstr = []
    outputstr.append("VERSION .7\n")
    outputstr.append("FIELDS x y z intensity\n")
    outputstr.append("SIZE 4 4 4 4\n")
    outputstr.append("TYPE F F F F\n")
    outputstr.append("WIDTH ")
    outputstr.append(str(width * num_slices) + "\n")
    outputstr.append("HEIGHT ")
    outputstr.append(str(height) + "\n")
    outputstr.append("VIEWPOINT 0 0 0 1 0 0 0\n")
    outputstr.append("POINTS ")
    outputstr.append(str(len(obj)) + "\n")
    outputstr.append("DATA ascii\n")

    for i in obj:
        outputstr.append(str(i.x) + " " + str(i.y) + " " +
                         str(i.z) + " " + str(i.intensity) + "\n")

    f.write("".join(outputstr))
