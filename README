Ensure that you have the necessary slam data files for each robonode
test0 is included as an example. Then run
python main.py robotID
This will start up a robot node ensure that all of your nodes are online within 25 seconds (this can be adjusted in robonode)
RoboNodes will set themselves to wrapped once all nodes have completed their SLAM simulation


MESSAGE TYPES

Used at start to make sure all bots wait for others to start running
SYNC:SENDID

Sends a request to update other bots
PUSHREQ:SENDID

Response to PUSHREQ prompts updates beyond last seen frame
PUSHACK:SENDID:lastSeenFrame


Single Keyframe update
UPDATE:SENDID:BOTID:KEYFRAME:DATA

Request Data for a given Bot will trigger update calls for missing keyframes
PULLREQ:SENDID:BOTID:KEYFRAME



