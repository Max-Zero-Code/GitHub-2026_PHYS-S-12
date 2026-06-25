var config = {
  student_name: "Max Zero de Sousa Costa", // ie. John Doe
  student_year_sem: "Summer 2026", // ie. Fall 2025
  student_email: "maxcostas2009@gmail.com",
  project_name1: "Smart copper coil launcher", // ie. My Awesome Project
  project_name2: "Forest fire monitoring system", // ie. My Second Awesome Project
  project_name3: "My Third Digital Fabrication Project", // ie. My Third Awesome Project
  // ie. jdoe@college.harvard.edu

  background_color: "#290404",
  text_color: "#c85a10",
  text_color2: "#fffb06",
  accent_color: "#FFFFFF",

  // Make sure to add the @import from Google Fonts to style.css, ask if you need help!
  font_family: "Lato",
  code_font_family: "Roboto Mono"
};

document.title = `${config.student_name}'s PS70 Website`;

document.documentElement.style.setProperty(
  "--background-color",
  config.background_color
);
document.documentElement.style.setProperty(
  "--text-color",
  config.text_color
);
document.documentElement.style.setProperty(
  "--text-color2",
  config.text_color2
);
document.documentElement.style.setProperty(
  "--accent-color",
  config.accent_color
);
document.documentElement.style.setProperty(
  "--font-family",
  config.font_family
);
document.documentElement.style.setProperty(
  "--mono-font-family",
  config.code_font_family
);
document.getElementById("student-name").textContent = config.student_name;
document.getElementById("project-name1").textContent = config.project_name1;
document.getElementById("project-name2").textContent = config.project_name2;
document.getElementById("project-name3").textContent = config.project_name3;

document.querySelector("footer").innerHTML = `
  <a href="./index.html#final-project">Work</a>
  <a href="./about.html">About</a>

  <div id="contact-info">
    <a href="mailto:${config.student_email}">${config.student_email}</a>
  </div>
`;

document.querySelectorAll('#student-name').forEach(el => {
  el.innerHTML = `${config.student_name}`;
});