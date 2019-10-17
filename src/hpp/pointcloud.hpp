#include <librealsense/rs.hpp>
#include <cstdio>
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>


double yaw, pitch, lastX, lastY; int ml;
static void on_mouse_button(GLFWwindow * win, int button, int action, int mods)
{
    if(button == GLFW_MOUSE_BUTTON_LEFT) ml = action == GLFW_PRESS;
}
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_Q && action == GLFW_PRESS)
            glfwDestroyWindow(window);
            glfwTerminate();
}
static double clamp(double val, double lo, double hi) { return val < lo ? lo : val > hi ? hi : val; }
static void on_cursor_pos(GLFWwindow * win, double x, double y)
{
    if(ml)
    {
        yaw = clamp(yaw - (x - lastX), -120, 120);
        pitch = clamp(pitch + (y - lastY), -80, 80);
    }
    lastX = x;
    lastY = y;
}

// GLFWwindow * point_cloud_init(){
//        // Open a GLFW window to display our output
//        glfwInit();
//        GLFWwindow * win = glfwCreateWindow(640, 480, "pointcloud", nullptr, nullptr);
//        glfwSetCursorPosCallback(win, on_cursor_pos);
//        glfwSetMouseButtonCallback(win, on_mouse_button);
//        glfwSetKeyCallback(win, key_callback);
//        glfwMakeContextCurrent(win);
//        glfwPollEvents();
//        return win;
//}
int point_cloud(rs::device& dev)
{

    // Open a GLFW window to display our output
    glfwInit();
    GLFWwindow * win = glfwCreateWindow(640, 480, "pointcloud", nullptr, nullptr);
    glfwSetCursorPosCallback(win, on_cursor_pos);
    glfwSetMouseButtonCallback(win, on_mouse_button);
    glfwSetKeyCallback(win, key_callback);
    glfwMakeContextCurrent(win);
    while(!glfwWindowShouldClose(win))
    {
        // Wait for new frame data
        glfwPollEvents();
        dev.wait_for_frames();

        // Retrieve our images
        const uint16_t * depth_image = (const uint16_t *)dev.get_frame_data(rs::stream::depth);
        const uint8_t * color_image = (const uint8_t *)dev.get_frame_data(rs::stream::color);

        // Retrieve camera parameters for mapping between depth and color
        rs::intrinsics depth_intrin = dev.get_stream_intrinsics(rs::stream::depth);
        rs::extrinsics depth_to_color = dev.get_extrinsics(rs::stream::depth, rs::stream::color);
        rs::intrinsics color_intrin = dev.get_stream_intrinsics(rs::stream::color);
        float scale = dev.get_depth_scale();

        // Set up a perspective transform in a space that we can rotate by clicking and dragging the mouse
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(60, (float)640/480, 0.01f, 20.0f);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(0,0,0, 0,0,1, 0,-1,0);
        glTranslatef(0,0,+0.5f);
        glRotated(pitch, 1, 0, 0);
        glRotated(yaw, 0, 1, 0);
        glTranslatef(0,0,-0.5f);

        // We will render our depth data as a set of points in 3D space
        glPointSize(2);
        glEnable(GL_DEPTH_TEST);
        glBegin(GL_POINTS);

        for(int dy=0; dy<depth_intrin.height; ++dy)
        {
            for(int dx=0; dx<depth_intrin.width; ++dx)
            {
                // Retrieve the 16-bit depth value and map it into a depth in meters
                uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
                float depth_in_meters = depth_value * scale;

                // Skip over pixels with a depth value of zero, which is used to indicate no data
                if(depth_value == 0) continue;

                // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                rs::float2 depth_pixel = {(float)dx, (float)dy};
                rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
                rs::float3 color_point = depth_to_color.transform(depth_point);
                rs::float2 color_pixel = color_intrin.project(color_point);

                // Use the color from the nearest color pixel, or pure white if this point falls outside the color image
                const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
                if(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height)
                {
                    glColor3ub(255, 255, 255);
                }
                else
                {
                    glColor3ubv(color_image + (cy * color_intrin.width + cx) * 3);
                }

                // Emit a vertex at the 3D location of this depth pixel
                glVertex3f(depth_point.x, depth_point.y, depth_point.z);
            }
        }
        glEnd();

        glfwSwapBuffers(win);
    }

    return EXIT_SUCCESS;
}
