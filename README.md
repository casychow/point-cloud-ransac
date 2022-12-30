# Point Cloud Random Sample Consensus (RANSAC)

Generally speaking, a RANSAC algorithm randomly chooses a set amount of points in a data set. Using such points, they create a line, a plane, or a hyperplane representing the randomly chosen data set points. Afterwards, they calculate the distance of every other point in the data set and compares it against a distance threshold from the created line/plane/hyperplane to determine inliers or outliers.

Inliers are data set points not initially chosen by the RANSAC algorithm that fall in between the distance threshold bordering the created line/plane/hyperplane. Outliers are data set points that do not fall in between the distance threshold and the created line/plane/hyperplane.

## Method
The minimum number of required point cloud points to make a plane is 3 points. As a result, my RANSAC algorithm randomly chooses 3 points and determines the equation of a plane. This minimum number can be increased to include more points, but the absolute minimum number is 3. Users can also change the distance threshold mentioned above to affect how many inliers and outliers a RANSAC iteration will output.

The confidence or voting threshold is the number of inliers over the total number of point cloud points. If the confidence threshold has been reached, the RANSAC algorithm immediately breaks out and renders the output. If the confidence threshold has not been reached, the other limiting variable for the RANSAC algorithm is the maximum number of total iterations. If this number has been reached, the RANSAC algorithm will output the render with the largest confidence threshold.

## Output
The following render was recevied after running the algorithm for 3 minutes and 20 seconds with a total vote rating of 75% when the confidence threshold was set at 90% and the maximum iteration at 500.

The red-colored points indicate inlier point cloud points.
![](https://i.imgur.com/MWiTt6f.png)
