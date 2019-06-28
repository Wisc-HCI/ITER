#!/usr/bin/env python

'''
Train Block Classifier Script
Authr: Curt Henrichs
Date: 6-17-19

Trains neural network to classify small and large blocks.

Note: the generated model from this script is needed by a ros node. Thus this
script must be run before that node is invoked if changes are made to the
configuration file.

Requires src/config/block_features.yaml with appropriate training data
Writes to src/model/block_classifer.nn with pickled model
'''

import os
import yaml
import pickle

from sklearn.neural_network import MLPClassifier
from sklearn.model_selection import train_test_split


DATA_FILEPATH = os.path.join(os.path.dirname(__file__),'../src/config/block_features.yaml')
MODEL_FILEPATH = os.path.join(os.path.dirname(__file__),'../src/model/block_classifer.nn')


# load config file
# formatted as:
#   - small_blocks: [],
#   - large_blocks: []
#
# where arrays contain elements of:
#   - [ratio,primary axis, primary rotation]
fin = open(DATA_FILEPATH,'r')
data = yaml.safe_load(fin)
fin.close()

# format data into dataset
features = []
labels = []
features += data['small_blocks']
labels += [[0,1] for i in range(0,len(data['small_blocks']))]
features += data['large_blocks']
labels += [[1,0] for i in range(0,len(data['large_blocks']))]
features += data['counter_cases']
labels += [[0,0] for i in range(0,len(data['counter_cases']))]

train_features, test_features, train_labels, test_labels = train_test_split(features,labels,shuffle=True,test_size=0.25)

# train model
network = MLPClassifier(solver='lbfgs',alpha=1e-5,hidden_layer_sizes=(10,),activation='relu',verbose=True)
model  = network.fit(train_features,train_labels)

# scoring
#score = model.score(test_features,test_labels)
#print 'Score: {0}'.format(score)

# save model
fout = open(MODEL_FILEPATH,'w')
pickle.dump(model,fout)
fout.close()
