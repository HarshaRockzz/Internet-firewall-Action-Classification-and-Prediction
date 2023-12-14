# PR Project - Internet firewall Action Classification and Prediction
### This is the project in which we have trained a model to classify and predict the Internet firewall action, and this classification and prediction is based on the data-setcollected from the ‘UCI machine learning Repository’. Here we are going to split the data into appropriate ratio i.e., 3:1, and first part is used to train the model and another part is to test the model. According to the data we can say that it’s multiclass classification, there are four actions given in the data-set, which can be understood as four classes.
# Data-set Information
### No. of Attributes = 12
### No. of Instances >= 1000
### Attributes are –
    Source Port
    Destination Port
    NAT Source Port
    NAT Destination Port
    Action – Target Attribute
    Bytes
    Bytes Sent
    Bytes Received
    Packets
    Elapsed Time (sec)
    pkts_sent
    pkts_received
# Random Forest Classifier
### Random forest, like its name implies, consists of a large number of individual decision trees that operate as an ensemble. Each individual tree in the random forest spits out a class prediction and the class with the most votes becomes our model’s prediction.

<p align = "center">
  <img src="https://user-images.githubusercontent.com/56586584/144798799-3fb538b2-32a8-4968-9bd8-1e344b375bd6.jpg" alt="Random forest Classifier">
</p>

### The fundamental concept behind random forest is a simple but powerful one — the wisdom of crowds. In data science speak, the reason that the random forest model works so well is:
## 'A large number of relatively uncorrelated models (trees) operating as a committee will outperform any of the individual constituent models.'
