- - - - - - - - - - 
GENERAL INFO
- - - - - - - - - - 

This package let's you train an audio classifier, classify live audio input and create and visualize audio classes.

- - - - - - - - - - 
EXECUTION INSTRUCTIONS
- - - - - - - - - - 

Adding a new class:
roslaunch audio_features_classification generate_class.launch

Visualizing the classes:
roslaunch audio_features_classification viz.launch

Training the classifier:
roslaunch audio_features_classification train.launch

Testing the classifier:
roslaunch audio_features_classification classifier.launch

- - - - - - - - - - 
PARAMETERS
- - - - - - - - - - 

You can (must) change the parameters found in the parameters.yaml file inder the config directory for each of the four executables based on your needs.

For example, you always need to change the "current_class" parameter based on the name of the class you want to generate.

You always have to add all the classes you want to train your classifier with, in the "classes" parameter, according to the names you have given during the class generation procedure.

- - - - - - - - - - 
SIDE NOTE
- - - - - - - - - - 

Along with generate_class.launch and classifier.launch you need to run the executable from audio_features_extraction package.