/**
 * @file iCubModels.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia Released under the terms of the Creative Commons
 * Attribution Share Alike 4.0 International
 */

#ifndef ICUB_MODELS_ICUB_MODELS_H
#define ICUB_MODELS_ICUB_MODELS_H

#include <string>
#include <unordered_set>

namespace iCubModels
{
/**
 * @brief Get the folder where the models are installed.
 * @return a string containing the folder where all the models have been installed.
 */
std::string getModelsPath();

/**
 * @brief Get the available robots.
 * @return an unordered set containing all the available robots.
 */
std::unordered_set<std::string> getRobotNames();

/**
 * @brief Get the file location associated to the a given model.
 * @param modelName a string containing the the name of the model
 * @return a string containing the path of the model.
 * @note If the modelName cannot be found among the installed one, an empty string is returned.
 */
std::string getModelFile(const std::string& modelName);

} // namespace iCubModels

#endif // ICUB_MODELS_ICUB_MODELS_H
