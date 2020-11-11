#version 330 core
in vec3 fragmentPos;
in vec3 fragmentColor;
in vec3 fragmentNormal;

out vec3 FragColor;

uniform vec3 viewPos;

void main(){
  vec3 lightColor = vec3(0.5,0.5,0.5);
  vec3 lightPos = vec3(10,10,10);

  vec3 norm = normalize(fragmentNormal);
  vec3 lightDir = normalize(lightPos - fragmentPos);
  vec3 viewDir = normalize(viewPos - fragmentPos);
  vec3 reflectDir = reflect(-lightDir, norm);

  // ambient
  float ambientStrength = 0.8;
  vec3 ambient = ambientStrength * fragmentColor;

  // diffuse
  float diffStrength = max(dot(norm, lightDir), 0.01);
  vec3 diffuse = diffStrength * lightColor;

  // specular
  float specStrength = pow(max(dot(viewDir, reflectDir),0), 32);
  vec3 specular = 0.05 * specStrength * lightColor;


  FragColor = ambient + diffuse + specular;
}
