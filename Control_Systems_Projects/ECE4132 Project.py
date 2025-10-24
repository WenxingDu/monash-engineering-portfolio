""" ECE4132 Project Code: Task 2 and 3
Authors: Thomas Huang, Yixu Wang, Wenxing Du
Contains the code that defines:
- Task 2 The drone sytem
- Task 3 Identification of beta, kappa and price values 
- Relevant plots for each task
"""
import numpy as np
import random
import matplotlib.pyplot as plt

#Task 2: Defining the system
#Logistic Function
def probYes(pi:int) -> int:
    """ Applies the logistic function to the price returning a probability of deciding yes
    @param pi: Output of the controller, payment price
    @return: The output of the logistic function/ probability of deciding yes.
    """
    return 0.01 + (0.98/(1+np.exp(-0.005*(pi-1200))))

#Implementation of the control system
def droneSystem(droneTotal:int,r:int,alpha:int,beta:int,kappa:int,totalTime:int,priceInit = 0) -> tuple[list,list,list]:
    """
    @param droneTotal: Number of drones available
    @param r: Reference/ drone target
    @param alpha: Controller variable alpha
    @param beta: Controller variable beta
    @param kappa: Controller variable kappa
    @param totalTime: The total run time of the system
    @param priceInit: Optional parameter for specifying initial price
    @return: Arrays that hold outputs at each time step
    - y: Records the total number of yes per time step 
    - e: Records the error per time step 
    - pi: Records the proposed price per time step
    """
    y = np.zeros(totalTime) 
    e = np.zeros(totalTime) 
    pi = np.zeros(totalTime) 
    pi[0] = priceInit
    for k in range(1,totalTime):
        #Error
        e[k] = r - y[k-1]

        #Controller
        pi[k] = beta*pi[k-1] + kappa*(e[k]-alpha*e[k-1])

        #Plant
        prob = probYes(pi[k])
        #Samples based on output of the logistic function for a total of the number of drone owners
        yi = random.choices([0,1],weights= [1-prob,prob],k = droneTotal)
        y[k] = sum(yi)
    return y,e,pi

#Task 3: Determining parameter values 
#Defining parameters 
droneTotal = 315
r = 140
alpha = -4.01
totalTime = 200

#Varying Kappa values, fixing beta
beta = 1
fig, ax = plt.subplots(3, 1, figsize=(9,9))
fig.supxlabel("Time")
fig.suptitle("Error over time for various kappa")
fig1, ax1 = plt.subplots(3, 1, figsize=(9,9))
fig1.supxlabel("Time")
fig1.suptitle("Varying kappa values for price")

#Iterating over kappa values 
kappavals = [0.01,0.05,0.9]
for i in range(len(kappavals)):
    y,e,pi = droneSystem(droneTotal,r,alpha,beta,kappavals[i],totalTime)
    #Plot of price for given kappa value
    ax1[i].plot(pi)
    ax1[i].set_title("kappa = " + str(kappavals[i]))
    ax1[i].set_ylabel("Price[cents]")
    ax[i].plot(e)
    ax[i].set_title("kappa = " + str(kappavals[i]))
    ax[i].set_ylabel("Error[drones]")

#Varying beta values, fixing kappa
kappa = 0.05 
fig2, ax2 = plt.subplots(3, 1, figsize=(9,9))
fig3, ax3 = plt.subplots(3, 1, figsize=(9,9))
fig3.supxlabel("Time")
fig3.suptitle("Error over time for various beta")
fig2.supxlabel("Time")
fig2.suptitle("Varying beta values for price")
betavals = [0.5,1,1.1]

#Iterating over beta values 
for i in range(len(betavals)):
    y,e,pi = droneSystem(droneTotal,r,alpha,betavals[i],kappa,totalTime)
    #Plot of price for given beta value
    ax2[i].plot(pi)
    ax2[i].set_title("beta = " + str(betavals[i]))
    ax2[i].set_ylabel("Price[cents]")
    ax3[i].plot(e)
    ax3[i].set_title("beta = " + str(betavals[i]))
    ax3[i].set_ylabel("Error[drones]")

y,e,pi = droneSystem(droneTotal,r,alpha,0.99,kappa,totalTime)
fig5, ax5 = plt.subplots(2, 1, figsize=(9,9))
ax5[0].plot(pi)
ax5[0].set_ylabel("Price[cents]")
ax5[0].set_title("Price over time for beta = " + str(0.99))
ax5[1].plot(e)
ax5[1].set_title("Error over time for beta = " + str(0.99))
ax5[1].set_ylabel("Error[drones]")
fig5.supxlabel("Time")
#Plot of final values for beta and kappa
#Controls no. drones relative to reference/ growth of price. Must be 1 for 0 error
beta = 1
#Kappa controls dynamics 
kappa = 0.05 

y,e,pi = droneSystem(droneTotal,r,alpha,beta,kappa,totalTime)

#Plotting result of one trial 
fig4, ax4 = plt.subplots(3, 1, figsize=(9,9))
ax4[0].plot(y)
ax4[0].set_ylabel("Yes drones[drones]")
ax4[0].set_title("y")
ax4[1].plot(e)
ax4[1].set_ylabel("Error[drones]")
ax4[1].set_title("e")
ax4[2].plot(pi)
ax4[2].set_ylabel("Price[cents]")
ax4[2].set_title("pi")
fig4.supxlabel("Time")
fig4.suptitle("One iteration")

#Mean and standard deviation of the steady state value taking last 50 time steps
print(f"For one trial: The mean price is {np.mean(pi[-50:-1])/100:.3f} dollars with steady state standard deviation {np.std(pi[-50:-1])/100:.3f}")
print(f"For one trial: The mean error is {np.mean(e[-50:-1]):.3f} drones with steady state standard deviation {np.std(e[-50:-1]):.3f}")

plt.show()