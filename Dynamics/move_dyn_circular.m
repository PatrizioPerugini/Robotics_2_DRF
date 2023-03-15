robot = rigidBodyTree('DataFormat', 'column', 'MaxNumBodies', 3);
%initial position of the robot 
q0_1=1.7610;
q0_2=-1.9512;
q0_3=0.9756;

q11 = ris.q1;
q22 = ris.q2;
q33 = ris.q3;

q1=[];
q2=[];
q3=[];
for i=1:length(q11(1))
    for j = 1:length(q11(1,1))
        v = 0;
        for k = 1:length(q11(1,1,:))
            v = q11(i,j,k);
            q1(k)=v;
            v = q22(i,j,k);
            q2(k)=v;
            v = q33(i,j,k);
            q3(k)=v;
           % disp(v)
        end
    end
    
end

q1 = q1';
q2=q2';
q3=q3';



%% Drawing the robot

L1 = 1;
L2 = 1;
L3 = 1;



% Adding the link 1
body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
joint.HomePosition = q0_1;
setFixedTransform(joint, trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');

% Adding the link 2
body = rigidBody('link2');
joint = rigidBodyJoint('joint2', 'revolute');
joint.HomePosition = q0_2;
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');

% Adding the link3
body = rigidBody('link3');
joint = rigidBodyJoint('joint3','revolute');
joint.HomePosition = q0_3;
setFixedTransform(joint, trvec2tform([L2, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link2');

%Adding the end-effector

body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L3, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link3');


% This block of code is needed to create cylinders 
for i = 1:robot.NumBodies -1
   ang = pi/2;
   collisionObj = collisionCylinder(0.05,1);
   mat = axang2tform([0 1 0 ang]);
   mat(1,4) = L1/2;
   collisionObj.Pose =mat;
   addCollision(robot.Bodies{i},collisionObj);
   
end

qs = [q1 q2 q3];
count = length(qs(:,1));
disp(count);


%% Defining the circular trajectory for the display
t = (0:0.1:4)'; % Time
center = [1 1.5 0];
radius = 0.5;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];


figure
show(robot,qs(1,:)','Collisions','on','Visuals','off');

view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
axis([-2 3 -2 3])


framesPerSecond = count/simulation_time;
% framesPerSecond = simulation_time/(count-1);
disp(framesPerSecond);
r = rateControl(framesPerSecond);
for i = 1:count-1
    show(robot,qs(i,:)','Collisions','on','Visuals','off','PreservePlot',false);
    drawnow
    waitfor(r);
end


