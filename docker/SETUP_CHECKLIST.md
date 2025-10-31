# Setup Checklist: Docker Hub for Orin Images

Use this checklist to set up Docker Hub integration for your Jetson Orin images.

---

## ☑️ Prerequisites Setup

- [ ] Docker installed on your development machine
- [ ] Access to RoBorregos Docker Hub account (username and password)
- [ ] Jetson Orin device available for testing (or access to one)
- [ ] Git repository cloned: `git clone https://github.com/RoBorregos/home2.git`

---

## ☑️ Initial Configuration (One-time Setup)

### 1. Docker Hub Authentication

- [ ] Login to Docker Hub:
  ```bash
  docker login
  # Enter roborregos credentials
  ```
- [ ] Verify login:
  ```bash
  docker info | grep Username
  ```

### 2. Make Scripts Executable

- [ ] Navigate to repository:
  ```bash
  cd /path/to/home2
  ```
- [ ] Make scripts executable:
  ```bash
  chmod +x docker/push-orin-images.sh
  chmod +x docker/pull-orin-images.sh
  chmod +x docker/list-orin-images.sh
  ```
- [ ] Verify scripts are executable:
  ```bash
  ls -l docker/*.sh
  ```

### 3. GitHub Actions Setup (Optional but Recommended)

- [ ] Go to GitHub repository settings:
  - URL: `https://github.com/RoBorregos/home2/settings/secrets/actions`
- [ ] Add `DOCKER_USERNAME` secret:
  - Click "New repository secret"
  - Name: `DOCKER_USERNAME`
  - Value: Your Docker Hub username
- [ ] Add `DOCKER_PASSWORD` secret:
  - Click "New repository secret"
  - Name: `DOCKER_PASSWORD`
  - Value: Your Docker Hub password or access token
- [ ] Verify workflow file exists:
  ```bash
  ls -l .github/workflows/docker-orin.yml
  ```

---

## ☑️ First Build and Push (Testing)

### Method 1: Using Scripts (Recommended for First Time)

- [ ] Choose a test version (e.g., `test-$(date +%Y%m%d)`)
- [ ] Build and push images:
  ```bash
  ./docker/push-orin-images.sh test-$(date +%Y%m%d)
  ```
- [ ] Monitor the build process (this may take 2-4 hours)
- [ ] Verify images on Docker Hub:
  - Visit: `https://hub.docker.com/r/roborregos/home2/tags`
  - Look for your test version tags

### Expected Output:

```
✓ Step 1: Base L4T Image - Built and pushed
✓ Step 2: HRI L4T Images - Built and pushed
✓ Step 3: Manipulation Jetson Image - Built and pushed
✓ Step 4: Navigation Jetson Image - Built and pushed
✓ Step 5: Vision Jetson Image - Built and pushed
```

---

## ☑️ Verification on Jetson Device

- [ ] SSH into Jetson Orin device
- [ ] Clone repository (if not already):
  ```bash
  git clone https://github.com/RoBorregos/home2.git
  cd home2
  ```
- [ ] Pull test images:
  ```bash
  ./docker/pull-orin-images.sh test-$(date +%Y%m%d)
  ```
  _Replace with your actual test version_
- [ ] Verify images are pulled:
  ```bash
  docker images | grep roborregos/home2
  ```
- [ ] Test one container:
  ```bash
  cd docker/manipulation
  docker compose -f docker-compose-jetson.yaml up
  ```
- [ ] Verify container works correctly
- [ ] Stop container:
  ```bash
  docker compose -f docker-compose-jetson.yaml down
  ```

---

## ☑️ Production Deployment Setup

### Create First Official Release

- [ ] Clean up test images locally:
  ```bash
  cd docker/
  make clean
  ```
- [ ] Build production version:
  ```bash
  ./push-orin-images.sh v1.0.0
  ```
  _Or use your own versioning scheme_
- [ ] Verify on Docker Hub
- [ ] Test on Jetson with production version:
  ```bash
  ./docker/pull-orin-images.sh v1.0.0
  ```

### Set Up Version Control

- [ ] Create Git tag for release:
  ```bash
  git tag -a v1.0.0 -m "Initial Docker Hub release for Orin images"
  git push origin v1.0.0
  ```
- [ ] If GitHub Actions is set up, verify automatic build triggered
- [ ] Check GitHub Actions tab: `https://github.com/RoBorregos/home2/actions`

---

## ☑️ Documentation Review

- [ ] Read Quick Reference guide:
  ```bash
  cat docker/QUICK_REFERENCE.md
  ```
- [ ] Bookmark Docker Hub Guide:
  ```bash
  open docker/DOCKER_HUB_GUIDE.md
  ```
- [ ] Share documentation with team members
- [ ] Add notes about your specific workflow or conventions

---

## ☑️ Team Onboarding

### For New Team Members:

- [ ] Share this checklist
- [ ] Provide Docker Hub credentials (if applicable)
- [ ] Show Quick Reference guide location
- [ ] Explain version numbering convention
- [ ] Demonstrate pulling and running images

### Create Team Documentation:

- [ ] Document your versioning strategy (semantic or date-based)
- [ ] Document release process (who can push, when to release)
- [ ] Document testing requirements before pushing
- [ ] Set up communication channel for deployment notifications

---

## ☑️ Maintenance Schedule

### Weekly:

- [ ] Check for new commits that need image updates
- [ ] Review Docker Hub storage usage
- [ ] Clean old/unused image tags if needed

### Before Competitions/Demos:

- [ ] Build and tag stable version
- [ ] Test all modules on Jetson
- [ ] Create backup of working images
- [ ] Document exact versions being used

### After Major Changes:

- [ ] Build and push new version
- [ ] Update documentation if workflow changed
- [ ] Notify team of new version availability
- [ ] Test on Jetson before marking as stable

---

## ☑️ Troubleshooting Checklist

If builds fail:

- [ ] Check Docker daemon is running: `docker ps`
- [ ] Verify disk space: `df -h`
- [ ] Clear Docker cache: `docker builder prune -a`
- [ ] Check base image availability: `docker pull dustynv/l4t-pytorch:r36.4.0`

If pushes fail:

- [ ] Verify Docker Hub login: `docker login`
- [ ] Check network connection
- [ ] Try pushing one image at a time
- [ ] Check Docker Hub storage limits

If pulls fail on Jetson:

- [ ] Verify Jetson has internet connection
- [ ] Check available disk space on Jetson
- [ ] Verify image tag exists on Docker Hub
- [ ] Try pulling with explicit registry: `docker pull docker.io/roborregos/home2:l4t_base-v1.0.0`

---

## ☑️ Success Criteria

You've successfully set up Docker Hub integration when:

- [ ] ✅ You can build all 5 images locally
- [ ] ✅ You can push all images to Docker Hub
- [ ] ✅ Images appear in Docker Hub repository with correct tags
- [ ] ✅ You can pull images on Jetson device
- [ ] ✅ Pulled images run correctly on Jetson
- [ ] ✅ Team members can pull and use images
- [ ] ✅ GitHub Actions builds automatically (if configured)
- [ ] ✅ Version management system is in place

---

## 📝 Notes Section

Use this space for your specific notes:

**Our versioning convention:**

- [ ] Documented: ****************\_\_\_****************

**Release schedule:**

- [ ] Documented: ****************\_\_\_****************

**Responsible team members:**

- [ ] Build/Push: ****************\_\_\_****************
- [ ] Testing: ****************\_\_\_****************
- [ ] Documentation: ****************\_\_\_****************

**Important commands for our workflow:**

```bash
# Add your specific commands here

```

**Known issues or gotchas:**

-

---

## 🎉 Completion

- [ ] All checklist items completed
- [ ] Images successfully built and pushed
- [ ] Images successfully pulled and tested on Jetson
- [ ] Documentation shared with team
- [ ] Workflow established and documented

**Date completed:** ******\_\_\_******
**Completed by:** ******\_\_\_******
**Version deployed:** ******\_\_\_******

---

## 📞 Support

If you encounter issues:

1. Check the troubleshooting section in `DOCKER_HUB_GUIDE.md`
2. Review error messages carefully
3. Search existing GitHub issues
4. Create new issue if problem persists
5. Contact DevOps team lead

---

**Next Steps After Completion:**

1. Schedule regular image updates
2. Monitor Docker Hub storage
3. Keep documentation updated
4. Train new team members
5. Refine workflow based on experience

Good luck with your deployment! 🚀
