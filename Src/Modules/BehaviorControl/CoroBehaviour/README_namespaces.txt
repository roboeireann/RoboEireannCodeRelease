Here is how we use namespaces for CoroBehaviours.

Almost everything in the CoroBehaviour directory tree is in the CoroBehaviour
namespace. This allows us to use some of the same class names as BHuman code
where that is the most sensible name to use.

Behaviours which are specific to a particular RoboCup event, normally
technical challenges, are placed in an RCyyyy namespace (and directory) where
yyyy is the RoboCup year.

Behaviours developed in a particular year are, for the time being, put in an
REyyyy namespace, e.g. RE2022. In general the soccer game behaviours and
utility behaviours that may be used again (e.g. KickLengthCalibration)
fall into this category.

Why bother with the year? The idea is so that we can compare a behaviour
from one year with another. How practical this idea ends up being remains
to be seen.