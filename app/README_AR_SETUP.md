# ARKit Setup Instructions

To enable ARKit functionality, you must add the Camera Usage Description to your project's `Info.plist`.

1.  Open `app/robot_arm_teleop/robot_arm_teleop.xcodeproj` in Xcode.
2.  Select the `robot_arm_teleop` target.
3.  Go to the **Info** tab.
4.  Add a new key: `Privacy - Camera Usage Description` (Raw key: `NSCameraUsageDescription`).
5.  Set the value to: "Camera access is required for AR tracking."

Without this, the app will crash immediately upon launch.
