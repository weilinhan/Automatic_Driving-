# psaf_sign_detection

## About

This node is responsible for detecting signs in camera images.
It uses a custom trained yolo v5 for sign detection (@weilin.han).
Sign IDs can be found in [sign_id.hpp](./include/psaf_sign_detection/sign_id.hpp).

## **Updating Classes**

**If you change the classes file used by the node, you need to update [sign_id.hpp](./include/psaf_sign_detection/sign_id.hpp) to match!**

## **Updating Models**

This node only works with yolo v5.
If you want to use antother model, take a look at the [sign_detector](./include/psaf_sign_detection/sign_detector.hpp) class.
It is responsible for using the network to detect signs and you will have to modify it.
