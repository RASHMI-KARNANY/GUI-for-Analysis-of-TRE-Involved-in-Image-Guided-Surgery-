# GUI-for-Analysis-of-TRE-Involved-in-Image-Guided-Surgery

This Graphical User Interface or GUI is primarily designed to calculate the magnitude of Target Registration Error (TRE) to help surgeons giving them an approximate idea of error for any Target during surgeries. Fiducials are the objects that help to register different data points. A minimum of three Markers is required to compute TRE for any target. The position of these Markers is decided by the operator and with the help of this GUI, the operator can try various positions of Markers and select the appropriate one having the least value of TRE. This GUI also provides a 3-Dimensional plot based on the coordinate system where one can visualise the position of the Markers and Targets added.

With further upgradations in this GUI, now we can also load the CT scan images which are known as DICOM images and have an approximate idea of the localization of Markers and Targets with respect to the human anatomy. Generally, three or more such markers are localized in all the spaces under study to properly register the specified object or target.


## Fiducial Localization Error (FLE): 
Point-based Registration involves localizing
fiducial points in the two spaces before registering them. The localization of the
fiducial points is known to be inaccurate, and the error made in this process is
termed Fiducial Localization Error (FLE).

## Fiducial Registration Error (FRE):
Due to the presence of FLE during localizing the
fiducials, the corresponding fiducial points do not align perfectly after the
registration, and the error in the alignment is called Fiducial Registration Error
(FRE). Practically, FRE and TRE can’t be calculated. So, in order to calculate FRE we
should know FLE and then we can apply the following equation to compute FRE,
where N is the number of fiducials used.

## Target Registration Error (TRE): 
The targets also do not align properly like the
fiducials after registration because of the presence of FLE. The Euclidean distance
between corresponding targets after registration is termed Target Registration
Error (TRE).
