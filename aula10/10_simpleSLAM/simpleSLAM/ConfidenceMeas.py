# adapted from Sebastian Thrun, Programming a Robotic Car, udacity

# -----------
# User Instructions
#
# Modify the previous code to adjust for a highly
# confident last measurement. Do this by adding a
# factor of 5 into your Omega and Xi matrices.


 
from math import *
from matrix import matrix
import random


#===============================================================
#
# SLAM in a rectolinear world (we avoid non-linearities)
#	  
# 
#===============================================================





# ######################################################################
# ######################################################################
# ######################################################################


# Including the 5 times multiplier, your returned mu should now be:
#
# [[-3.0],
#  [2.179],
#  [5.714],
#  [6.821]]



############## MODIFY CODE BELOW ##################

def doit(initial_pos, move1, move2, Z0, Z1, Z2):

	# 1. Pesos das medições
	w0 = 1
	w1 = 1
	w2 = 5  # 5x mais confiança na última medição

	# 2. Jacobiana (J) segundo a relação com o Landmark
	J = matrix([
		[1, 0, 0, 0],   # Movimento x0 -> x1
		[-1, 1, 0, 0],   # Movimento x1 -> x2
		[0, -1, 1, 0],   # Movimento x2
		[1, 0, 0, -1],   # Landmark em x1
		[0, 1, 0, -1],   # Landmark em x2
		[0, 0, 1, -1]    # Landmark em x0
	])

	# 3. Vetor de erro (b)
	b = matrix([
		[initial_pos],
		[move1],
		[move2],
		[-Z0],
		[-Z1],
		[-Z2]
	])

	# 4. Matriz de pesos (W) - Diagonal
	W = matrix([
		[1, 0, 0, 0, 0, 0],  # Movimento x0 -> x1 (peso normal)
		[0, 1, 0, 0, 0, 0],  # Movimento x1 -> x2 (peso normal)
		[0, 0, 1, 0, 0, 0],  # Movimento x2        (peso normal)
		[0, 0, 0, w0, 0, 0],  # Landmark Z0 (peso alterado se w0 mudar)
		[0, 0, 0, 0, w1, 0],  # Landmark Z1 (peso alterado se w1 mudar)
		[0, 0, 0, 0, 0, w2]   # Landmark Z2 (peso alterado se w2 mudar)
	])

	# 5. Computar H e xi
	H = J.transpose() * W * J
	xi = J.transpose() * W * b

	# 6. Resolver o sistema para encontrar mu (posições corrigidas)
	mu = H.inverse() * xi

	return mu

print(doit(-3, 5, 3, 10, 5, 1))



