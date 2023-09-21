/*******************************************************************************
 * Copyright 2021 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
#include <modal_pipe.h>
#include "voxl_mpa_to_ros2/interfaces/pose_vel_6dof_interface.h"
#include "voxl_mpa_to_ros2/utils/common_utils.h"
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <px4_ros_com/frame_transforms.h>

static void _helper_cb(
    __attribute__((unused))int ch, 
                           char* data, 
                           int bytes, 
                           void* context);

PoseVel6DOFInterface::PoseVel6DOFInterface(
    rclcpp::Node::SharedPtr nh,
    const char *    name) :
    GenericInterface(nh, name)
{

    pipe_client_set_simple_helper_cb(m_channel, _helper_cb, this);

    if(pipe_client_open(m_channel, name, PIPE_CLIENT_NAME,
                EN_PIPE_CLIENT_SIMPLE_HELPER | EN_PIPE_CLIENT_AUTO_RECONNECT,
                POSE_6DOF_RECOMMENDED_READ_BUF_SIZE)){
        pipe_client_close(m_channel);//Make sure we unclaim the channel
        throw -1;
    }

}

void PoseVel6DOFInterface::AdvertiseTopics(){

    char topicName[64];

    sprintf(topicName, "%s", m_pipeName);
    pose_vel_6dof_pub_ = m_rosNodeHandle->create_publisher<voxl_msgs::msg::Poseveldof>
        (topicName, rclcpp::SensorDataQoS());

    m_state = ST_AD;

}

void PoseVel6DOFInterface::StopAdvertising(){

    pose_vel_6dof_pub_.reset();

    m_state = ST_CLEAN;

}

int PoseVel6DOFInterface::GetNumClients(){
    return pose_vel_6dof_pub_->get_subscription_count();
}

// called when the simple helper has data for us
static void _helper_cb(__attribute__((unused))int ch, char* data, int bytes, void* context)
{
	if(bytes<=0){
        printf("No bytes through pipe");
        return;
    }
    int n_packets;
	// check for 4dof pose packets

    pose_vel_6dof_t* pose_array = pipe_validate_pose_vel_6dof_t(data, bytes, &n_packets);
    if(pose_array==NULL){
        printf("ERROR, failed to validate pose_6dof_t packets on fixed pose input sink\n");
        return;
    }

    PoseVel6DOFInterface *interface = (PoseVel6DOFInterface *) context;
    if(interface->GetState() != ST_RUNNING) return;
    voxl_msgs::msg::Poseveldof& poseveldof = interface->GetPoseVel6DOFMsg();

    for(int i=0;i<n_packets;i++){
        poseveldof.timestamp_ns = pose_array[i].timestamp_ns;
        poseveldof.t_child_wrt_parent[0] = pose_array[i].T_child_wrt_parent[0];
        poseveldof.t_child_wrt_parent[1] = pose_array[i].T_child_wrt_parent[1];
        poseveldof.t_child_wrt_parent[2] = pose_array[i].T_child_wrt_parent[2];

        poseveldof.r_child_to_parent[0] = pose_array[i].R_child_to_parent[0][0];
        poseveldof.r_child_to_parent[1] = pose_array[i].R_child_to_parent[0][1];
        poseveldof.r_child_to_parent[2] = pose_array[i].R_child_to_parent[0][2];
        poseveldof.r_child_to_parent[3] = pose_array[i].R_child_to_parent[1][0];
        poseveldof.r_child_to_parent[4] = pose_array[i].R_child_to_parent[1][1];
        poseveldof.r_child_to_parent[5] = pose_array[i].R_child_to_parent[1][2];
        poseveldof.r_child_to_parent[6] = pose_array[i].R_child_to_parent[2][0];
        poseveldof.r_child_to_parent[7] = pose_array[i].R_child_to_parent[2][1];
        poseveldof.r_child_to_parent[8] = pose_array[i].R_child_to_parent[2][2];

        poseveldof.v_child_wrt_parent[0] = pose_array[i].v_child_wrt_parent[0];
        poseveldof.v_child_wrt_parent[1] = pose_array[i].v_child_wrt_parent[1];
        poseveldof.v_child_wrt_parent[2] = pose_array[i].v_child_wrt_parent[2];

        poseveldof.w_child_wrt_child[0] = pose_array[i].w_child_wrt_child[0];
        poseveldof.w_child_wrt_child[1] = pose_array[i].w_child_wrt_child[1];
        poseveldof.w_child_wrt_child[2] = pose_array[i].w_child_wrt_child[2];

        interface->pose_vel_6dof_pub_->publish(poseveldof);
    }
    return;
}
