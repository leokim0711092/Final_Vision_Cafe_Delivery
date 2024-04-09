# RoboCafé - Vision based cafe delivery

RoboCafé aims to revolutionize the café experience by introducing an automated, vision-based delivery system. The project employs cutting-edge technology to automate the delivery of beverages, utilizing a robotic arm to ensure precision and efficiency. The core idea is to streamline café operations and offer a novel, tech-driven service to customers.


## Workflow Overview:

1. Web Request Initiation: 
Customers initiate beverage orders through a user-friendly web interface. These requests are then forwarded to a Docker container, which acts as the central processing unit for the operation.

2. Perception and Analysis: 
The system employs an Intel D415 RGB camera to capture detailed visual data of the café environment. The Point Cloud Library (PCL) processes this data to understand the space and identify the precise location for cup placement.

3. Position Determination: 
A mathematical model, specifically the least square approximation, is used to slove the ideal circle equation. This calculation precisely determines where the robotic arm should place the beverage to ensure accuracy and stability.

4. Trajectory Planning and Execution: 
Utilizing MoveIt2, the system plans the trajectory for the UR3e robotic arm. This ensures the arm moves smoothly and accurately to place the coffee in the determined location, completing the delivery process.


## Run Locally

Clone the project

```bash
  git clone https://link-to-project
```

Go to the project directory

```bash
  cd my-project
```

Install dependencies

```bash
  npm install
```

Start the server

```bash
  npm run start
```


## Demo

[Insert gif or link to demo
](https://www.youtube.com/watch?v=-7of2jbOiUU)

## Authors

- [@leokim0711092](https://github.com/leokim0711092)
