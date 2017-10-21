
input = {}

input['height'] = 4
input['width'] = 4
input['channel'] = 2
input['height'] = 4
input['data'] = np.array(([[0, 4, -5, 5], [7, 0, 6, 3], [6, -6, 9, 1], [0, 0, 0, 7]], 
	[[5, 0, 9, 8], [4, -2, 0, 0], [1, 5, -9, 0], [0, 8, 5, 4]]))
input['batch_size'] = 1


output = {}
output['height'] = input['height']
output['width'] = input['width']
output['channel'] = input['channel']
output['batch_size'] = input['batch_size']
output['data'] = np.zeros(input['data'].shape)

# TODO: implement your relu forward pass here
# implementation begins
positive_indeces = input['data'] > 0
output['data'][positive_indeces] = input['data'][positive_indeces]
# implementation ends
