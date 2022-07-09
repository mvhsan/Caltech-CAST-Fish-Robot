function [USolve, DSolve, LSolve, RSolve] = solveInverseKinematics(UAngle, RAngle)
    %Lengths and variables in inches (Rough estimates)
    baseLength = 1.657;
    servoArmLength = 0.7382;
    linkageLength = 1.866;
    swashPlateLength = 2.348;
    middleHeight = 1.866;     

    %Solve for each angle
    USolve = solveSingleAngleInverseKinematics(UAngle);
    DSolve = solveSingleAngleInverseKinematics(-UAngle);
    RSolve = solveSingleAngleInverseKinematics(RAngle);
    LSolve = solveSingleAngleInverseKinematics(-RAngle);


    function IKOutputAngle = solveSingleAngleInverseKinematics(IKInputAngle)
        x_s = swashPlateLength/2 * cosd(IKInputAngle) - baseLength/2;
        y_s = middleHeight - swashPlateLength/2 * sind(IKInputAngle);
        beta = atand(y_s / x_s);
        R = sqrt((x_s)^2 + (y_s)^2);
        delta = acosd( ((servoArmLength)^2 + R^2 - linkageLength^2)/(2*servoArmLength*R));
        IKOutputAngle = (beta - delta);
    end
end



